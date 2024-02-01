/*
 * Control panel for IC Simulation
 *
 * OpenGarages
 *
 * craig@theialabs.com
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <getopt.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>

#define DATA_DIR "./data/"
#define DATA_FILE_SIZE 256
#define CAN_TRAFFIC_FILE_PATH DATA_DIR "sample-can.log"

#define SIGNAL_ID 392
#define SIGNAL_POS 0
#define SIGNAL_LEN 8

#define SPEED_ID 580
#define DEFAULT_SPEED_POS 3
#define SPEED_LEN 8

#define LEFT_SIGNAL_VALUE 1
#define RIGHT_SIGNAL_VALUE 2

#define SCREEN_WIDTH 835
#define SCREEN_HEIGHT 608

#define MAX_SPEED 90.0 // Limiter 260.0 is full guage speed
#define ACCEL_RATE 8.0 // 0-MAX_SPEED in seconds

int s; // socket
struct canfd_frame cf;
struct ifreq ifr;

char signal_state = 0;
int throttle = 0;
float current_speed = 0;
int turning = 0;
int currentTime;
int lastAccel = 0;
int lastTurnSignal = 0;

int play_id;

// Adds data dir to file name
// Uses a single pointer so not to have a memory leak
// returns point to data_files or NULL if append is too large
char *get_data(char *fname, char *data_file)
{
	if (strlen(DATA_DIR) + strlen(fname) > DATA_FILE_SIZE - 1)
		return NULL;
	strncpy(data_file, DATA_DIR, 255);
	strncat(data_file, fname, 255 - strlen(data_file));
	return data_file;
}

void send_pkt(int mtu)
{
	if (write(s, &cf, mtu) != mtu)
	{
		perror("write");
	}
}

void send_speed()
{
	int kph = (current_speed / 0.6213751) * 100;
	memset(&cf, 0, sizeof(cf));
	cf.can_id = SPEED_ID;
	cf.len = SPEED_LEN;
	if (kph == 0) // IDLE
	{
		cf.data[DEFAULT_SPEED_POS] = 1;
		cf.data[DEFAULT_SPEED_POS + 1] = rand() % 255 + 100;
	}
	else
	{
		cf.data[DEFAULT_SPEED_POS] = (char)(kph >> 8) & 0xff;
		cf.data[DEFAULT_SPEED_POS + 1] = (char)kph & 0xff;
	}
	send_pkt(CAN_MTU);
}

void send_turn_signal()
{
	memset(&cf, 0, sizeof(cf));
	cf.can_id = SIGNAL_ID;
	cf.len = SIGNAL_LEN;
	cf.data[SIGNAL_POS] = signal_state;
	send_pkt(CAN_MTU);
}

// Checks throttle to see if we should accelerate or decelerate the vehicle
void checkAccel()
{
	float rate = MAX_SPEED / (ACCEL_RATE * 100);
	// Updated every 10 ms
	if (currentTime > lastAccel + 10)
	{
		if (throttle < 0)
		{
			current_speed -= rate;
			if (current_speed < 1)
				current_speed = 0;
		}
		else if (throttle > 0)
		{
			current_speed += rate;
			if (current_speed > MAX_SPEED)
			{ // Limiter
				current_speed = MAX_SPEED;
			}
		}
		send_speed();
		lastAccel = currentTime;
	}
}

// Checks if turning and activates the turn signal
void checkTurn()
{
	if (currentTime > lastTurnSignal + 500)
	{
		if (turning < 0)
		{
			signal_state ^= LEFT_SIGNAL_VALUE;
		}
		else if (turning > 0)
		{
			signal_state ^= RIGHT_SIGNAL_VALUE;
		}
		else
		{
			signal_state = 0;
		}
		send_turn_signal();
		lastTurnSignal = currentTime;
	}
}

void play_can_traffic()
{
	char can2can[50];
	snprintf(can2can, 49, "%s=can0", ifr.ifr_name);
	if (execlp("canplayer", "canplayer", "-I", CAN_TRAFFIC_FILE_PATH, "-l", "i", can2can, NULL) == -1)
		printf("WARNING: Could not execute canplayer. No bg data\n");
}

void kill_child()
{
	kill(play_id, SIGINT);
}

void redraw_screen(SDL_Texture *base_texture, SDL_Renderer *renderer)
{
	SDL_RenderCopy(renderer, base_texture, NULL, NULL);
	SDL_RenderPresent(renderer);
}

void usage(char *msg)
{
	if (msg)
		printf("%s\n", msg);
	printf("Usage: controls <can>\n");
	exit(1);
}

int main(int argc, char *argv[])
{
	int opt;
	struct sockaddr_can addr;
	int running = 1;
	int enable_canfd = 1;
	SDL_Event event;
	SDL_Texture *base_texture = NULL;
	SDL_Renderer *renderer = NULL;
	char data_file[DATA_FILE_SIZE];

	/* Get options */
	while ((opt = getopt(argc, argv, "h?")) != -1)
	{
		switch (opt)
		{
		case 'h':
		case '?':
		default:
			usage(NULL);
			break;
		}
	}

	if (optind >= argc)
		usage("You must specify at least one can device");

	/* Create CAN socket */
	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
	{
		perror("socket");
		return 1;
	}

	addr.can_family = AF_CAN;

	strcpy(ifr.ifr_name, argv[optind]);
	if (ioctl(s, SIOCGIFINDEX, &ifr) < 0)
	{
		perror("SIOCGIFINDEX");
		return 1;
	}
	addr.can_ifindex = ifr.ifr_ifindex;

	if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
				   &enable_canfd, sizeof(enable_canfd)))
	{
		perror("error when enabling CAN FD support\n");
		return 1;
	}

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		perror("bind");
		return 1;
	}

	/* Play traffic */
	play_id = fork();
	if ((int)play_id == -1)
	{
		perror("Couldn't fork bg player\n");
		exit(-1);
	}
	else if (play_id == 0)
	{
		play_can_traffic(); // Shouldn't return
		exit(0);
	}
	atexit(kill_child);

	/* Setup GUI */
	SDL_Window *window = NULL;
	if (SDL_Init(SDL_INIT_VIDEO) < 0)
	{
		perror("SDL Could not initializes\n");
		exit(40);
	}
	
	window = SDL_CreateWindow("CANBus Control Panel", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
	if (window == NULL)
	{
		perror("Window could not be shown\n");
		exit(50);
	}

	renderer = SDL_CreateRenderer(window, -1, 0);
	SDL_Surface *image = IMG_Load(get_data("joypad.png", data_file));
	base_texture = SDL_CreateTextureFromSurface(renderer, image);
	SDL_RenderCopy(renderer, base_texture, NULL, NULL);
	SDL_RenderPresent(renderer);

	/* Main loop */
	while (running)
	{
		while (SDL_PollEvent(&event) != 0)
		{
			switch (event.type)
			{
			case SDL_QUIT:
				running = 0;
				break;
			case SDL_WINDOWEVENT:
				switch (event.window.event)
				{
				case SDL_WINDOWEVENT_ENTER:
				case SDL_WINDOWEVENT_RESIZED:
					redraw_screen(base_texture, renderer);
					break;
				}
				break;
			case SDL_KEYDOWN:
				switch (event.key.keysym.sym)
				{
				case SDLK_UP:
					throttle = 1;
					break;
				case SDLK_LEFT:
					turning = -1;
					break;
				case SDLK_RIGHT:
					turning = 1;
					break;
				break;
				}
				break;
			case SDL_KEYUP:
				switch (event.key.keysym.sym)
				{
				case SDLK_UP:
					throttle = -1;
					break;
				case SDLK_LEFT:
				case SDLK_RIGHT:
					turning = 0;
					break;
				}
				break;
			}
		}
		currentTime = SDL_GetTicks();
		checkAccel();
		checkTurn();
		SDL_Delay(5);
	}

	/* Cleanup */
	close(s);
	SDL_DestroyTexture(base_texture);
	SDL_FreeSurface(image);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();
}
