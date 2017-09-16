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
#include <time.h>
#include <pthread.h>
#include <termios.h>

#include "config.h"

#if !(DISABLE_SDL)
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#endif // !DISABLE_SDL

#ifndef DATA_DIR
#define DATA_DIR "./data/"
#endif
#define DEFAULT_CAN_TRAFFIC DATA_DIR "sample-can.log"

#define DEFAULT_DIFFICULTY 1
// 0 = No randomization added to the packets other than location and ID
// 1 = Add NULL padding
// 2 = Randomize unused bytes
#define DEFAULT_DOOR_ID 411
#define DEFAULT_DOOR_POS 2
#define DEFAULT_SIGNAL_ID 392
#define DEFAULT_SIGNAL_POS 0
#define DEFAULT_SPEED_ID 580
#define DEFAULT_SPEED_POS 3
#define CAN_DOOR1_LOCK 1
#define CAN_DOOR2_LOCK 2 
#define CAN_DOOR3_LOCK 4
#define CAN_DOOR4_LOCK 8
#define CAN_LEFT_SIGNAL 1
#define CAN_RIGHT_SIGNAL 2
#define ON 1
#define OFF 0
#define DOOR_LOCKED 0
#define DOOR_UNLOCKED 1
#define MAX_SPEED 150.0 // Limiter 260.0 is full gauge speed
#define ACCEL_RATE 8.0 // 0-MAX_SPEED in seconds

#if !(DISABLE_SDL)
#define SCREEN_WIDTH 835
#define SCREEN_HEIGHT 608
#define JOY_UNKNOWN -1
#define BUTTON_LOCK 4
#define PS3_BUTTON_LOCK 10
#define BUTTON_UNLOCK 5
#define PS3_BUTTON_UNLOCK 11
#define BUTTON_A 0
#define PS3_BUTTON_A 14
#define BUTTON_B 1
#define PS3_BUTTON_B 13
#define BUTTON_X 2
#define PS3_BUTTON_X 15 
#define BUTTON_Y 3
#define PS3_BUTTON_Y 12
#define BUTTON_START 7
#define PS3_BUTTON_START 3
#define AXIS_LEFT_V 0
#define PS3_AXIS_LEFT_V 0
#define AXIS_LEFT_H 1
#define PS3_AXIS_LEFT_H 1
#define AXIS_L2 2
#define PS3_AXIS_L2 12
#define AXIS_RIGHT_H 3
#define PS3_AXIS_RIGHT_H 3
#define AXIS_RIGHT_V 4
#define PS3_AXIS_RIGHT_V 2
#define AXIS_R2 5
#define PS3_AXIS_R2 13
#define PS3_X_ROT 4
#define PS3_Y_ROT 5
#define PS3_Z_ROT 6 // The rotations are just guessed
#define USB_CONTROLLER 0
#define PS3_CONTROLLER 1

int gButtonY = BUTTON_Y;
int gButtonX = BUTTON_X;
int gButtonA = BUTTON_A;
int gButtonB = BUTTON_B;
int gButtonStart = BUTTON_START;
int gButtonLock = BUTTON_LOCK;
int gButtonUnlock = BUTTON_UNLOCK;
int gAxisL2 = AXIS_L2;
int gAxisR2 = AXIS_R2;
int gAxisRightH = AXIS_RIGHT_H;
int gAxisRightV = AXIS_RIGHT_V;
int gAxisLeftH = AXIS_LEFT_H;
int gAxisLeftV = AXIS_LEFT_V;
// Acelleromoter axis info
int gJoyX = JOY_UNKNOWN;
int gJoyY = JOY_UNKNOWN;
int gJoyZ = JOY_UNKNOWN;

//Analog joystick dead zone
const int JOYSTICK_DEAD_ZONE = 8000;
int gLastAccelValue = 0; // Non analog R2
#endif // !DISABLE_SDL

int s; // socket
struct canfd_frame cf;
char *traffic_log = DEFAULT_CAN_TRAFFIC;

struct ifreq ifr;
int door_pos = DEFAULT_DOOR_POS;
int signal_pos = DEFAULT_SIGNAL_POS;
int speed_pos = DEFAULT_SPEED_POS;
int door_len = DEFAULT_DOOR_POS + 1;
int signal_len = DEFAULT_DOOR_POS + 1;
int speed_len = DEFAULT_SPEED_POS + 2;
int difficulty = DEFAULT_DIFFICULTY;

int lock_enabled = 0;
int unlock_enabled = 0;
char door_state = 0xf;
int throttle = 0;
float current_speed = 0;
int turning = 0;
int door_id, signal_id, speed_id;
struct timespec currentTime;
int current_ms = 0;
int lastAccel = 0;
int lastTurnSignal = 0;
char signal_state = 0;
int do_lock[4] = {0};
int do_unlock[4] = {0};

int seed = 0;
int debug = 0;
int play_pid;
int text_mode = 0;
int keyboard_mode = 0;

#if !(DISABLE_SDL)
int kk = 0;
char data_file[256];
SDL_GameController *gGameController = NULL;
SDL_Joystick *gJoystick = NULL;
SDL_Renderer *renderer = NULL;
SDL_Texture *base_texture = NULL;
int gControllerType = USB_CONTROLLER;

void kk_check(int);

// Adds data dir to file name
// Uses a single pointer so not to have a memory leak
// returns point to data_files or NULL if append is too large
char *get_data(char *fname) {
	if(strlen(DATA_DIR) + strlen(fname) > 255) return NULL;
	strncpy(data_file, DATA_DIR, 255);
	strncat(data_file, fname, 255-strlen(data_file));
	return data_file;
}
#endif // !DISABLE_SDL


void send_pkt(int mtu) {
	if(write(s, &cf, mtu) != mtu) {
		perror("write");
	}
}

// Randomizes bytes in CAN packet if difficulty is hard enough
void randomize_pkt(int start, int stop) {
	if (difficulty < 2) return;
	int i = start;
	for(;i < stop;i++) {
		if(rand() % 3 < 1) cf.data[i] = rand() % 255;
	}
}

void send_lock(char door) {
	door_state |= door;
	memset(&cf, 0, sizeof(cf));
	cf.can_id = door_id;
	cf.len = door_len;
	cf.data[door_pos] = door_state;
	if (door_pos) randomize_pkt(0, door_pos);
	if (door_len != door_pos + 1) randomize_pkt(door_pos + 1, door_len);
	send_pkt(CAN_MTU);
}

void send_unlock(char door) {
	door_state &= ~door;
	memset(&cf, 0, sizeof(cf));
	cf.can_id = door_id;
	cf.len = door_len;
	cf.data[door_pos] = door_state;
	if (door_pos) randomize_pkt(0, door_pos);
	if (door_len != door_pos + 1) randomize_pkt(door_pos + 1, door_len);
	send_pkt(CAN_MTU);
}

void check_locks() {
	for (int i = 0; i < 4; i++) {
		if (do_lock[i] == 1) {
			send_lock(1 << i);
			do_lock[i] = 0;
		}
		if (do_unlock[i] == 1) {
			send_unlock(1 << i);
			do_unlock[i] = 0;
		}
	}
}

void send_speed() {
	int kph = (int)current_speed * 100;
	memset(&cf, 0, sizeof(cf));
	cf.can_id = speed_id;
	cf.len = speed_len;
	cf.data[speed_pos+1] = (char)kph & 0xff;
	cf.data[speed_pos] = (char)(kph >> 8) & 0xff;
	if(kph == 0) { // IDLE
		cf.data[speed_pos] = 1;
		cf.data[speed_pos+1] = rand() % 255+100;
	}
	if (speed_pos) randomize_pkt(0, speed_pos);
	if (speed_len != speed_pos + 2) randomize_pkt(speed_pos+2, speed_len);
	send_pkt(CAN_MTU);
}

void send_turn_signal() {
	memset(&cf, 0, sizeof(cf));
	cf.can_id = signal_id;
	cf.len = signal_len;
	cf.data[signal_pos] = signal_state;
	if(signal_pos) randomize_pkt(0, signal_pos);
	if(signal_len != signal_pos + 1) randomize_pkt(signal_pos+1, signal_len);
	send_pkt(CAN_MTU);
}

// Checks throttle to see if we should accelerate or decelerate the vehicle
void check_accel() {
	float rate = MAX_SPEED / (ACCEL_RATE * 100);
	// Updated every 10 ms
	if(current_ms > lastAccel + 10) {
		if(throttle < 0) {
			current_speed -= rate;
			if(current_speed < 1) current_speed = 0;
		} else if(throttle > 0) {
			current_speed += rate;
			if(current_speed > MAX_SPEED) { // Limiter
				current_speed = MAX_SPEED;
			}
		}
		send_speed();
		lastAccel = current_ms;
	}
}

// Checks if turning and activates the turn signal
void check_turn() {
	if(current_ms > lastTurnSignal + 500) {
		if(turning < 0) {
			signal_state ^= CAN_LEFT_SIGNAL;
			signal_state &= ~CAN_RIGHT_SIGNAL;
		} else if(turning > 0) {
			signal_state ^= CAN_RIGHT_SIGNAL;
			signal_state &= ~CAN_LEFT_SIGNAL;
		} else {
			signal_state = 0;
		}
		send_turn_signal();
		lastTurnSignal = current_ms;
	}
}

#if !(DISABLE_SDL)
// Takes R2 joystick value and converts it to throttle speed
void accelerate(int value) {
	// Check dead zones
	if(gControllerType == PS3_CONTROLLER) {
		// PS3 works different.  the value range is 0-32k
		if (value < gLastAccelValue) {
			throttle = -1;
		} else if (value > gLastAccelValue) {
			throttle = 1;
		} else {
			throttle = 0;
		}
		gLastAccelValue = value;
	} else {
		if(value < -JOYSTICK_DEAD_ZONE) {
			throttle = -1;
		} else if(value > JOYSTICK_DEAD_ZONE) {
			throttle = 1;
		} else {
			throttle = 0;
		}
	}
}

// Check LEFT_V axis to see if we are turning
void turn(int value) {
	if(value < -JOYSTICK_DEAD_ZONE) {
		turning = -1;
		kk_check(SDLK_LEFT);
	} else if(value > JOYSTICK_DEAD_ZONE) {
		turning = 1;
		kk_check(SDLK_RIGHT);
	} else {
		turning = 0;
	}
}

void ud(int value) {
	if(value < -JOYSTICK_DEAD_ZONE) {
		kk_check(SDLK_UP);
	} else if(value > JOYSTICK_DEAD_ZONE) {
		kk_check(SDLK_DOWN);
	}
}

void kkpay() {
	printf("KK\n");
}

void kk_check(int k) {
	switch(k) {
		case SDLK_RETURN:
			if(kk == 0xa) kkpay();
			kk = 0;
			break;
		case SDLK_UP:
			kk = (kk < 2) ? kk+1 : 0;
			break;
		case SDLK_DOWN:
			kk = (kk > 1 && kk < 4) ? kk+1 : 0;
			break;
		case SDLK_LEFT:
			kk = (kk == 4 || kk == 6) ? kk+1 : 0;
			break;
		case SDLK_RIGHT:
			kk = (kk == 5 || kk == 7) ? kk+1 : 0;
			break;
		case SDLK_a:
			kk = kk == 9 ? kk+1 : 0;
			break;
		case SDLK_b:
			kk = kk == 8 ? kk+1 : 0;
			break;
		default:
			kk = 0;
	}
}

void redraw_gui() {
	SDL_RenderCopy(renderer, base_texture, NULL, NULL);
	SDL_RenderPresent(renderer);
}

// Maps the controllers buttons
void map_joy() {
	switch(gControllerType) {
		case USB_CONTROLLER:
			gButtonA = BUTTON_A;
			gButtonB = BUTTON_B;
			gButtonX = BUTTON_X;
			gButtonY = BUTTON_Y;
			gButtonStart = BUTTON_START;
			gButtonLock = BUTTON_LOCK;
			gButtonUnlock = BUTTON_UNLOCK;
			gAxisL2 = AXIS_L2;
			gAxisR2 = AXIS_R2;
			gAxisRightH = AXIS_RIGHT_H;
			gAxisRightV = AXIS_RIGHT_V;
			gAxisLeftH = AXIS_LEFT_H;
			gAxisLeftV = AXIS_LEFT_V;
			break;
		case PS3_CONTROLLER:
			gButtonA = PS3_BUTTON_A;
			gButtonB = PS3_BUTTON_B;
			gButtonX = PS3_BUTTON_X;
			gButtonY = PS3_BUTTON_Y;
			gButtonStart = PS3_BUTTON_START;
			gButtonLock = PS3_BUTTON_LOCK;
			gButtonUnlock = PS3_BUTTON_UNLOCK;
			gAxisL2 = PS3_AXIS_L2;
			gAxisR2 = PS3_AXIS_R2;
			gAxisRightH = PS3_AXIS_RIGHT_H;
			gAxisRightV = PS3_AXIS_RIGHT_V;
			gAxisLeftH = PS3_AXIS_LEFT_H;
			gAxisLeftV = PS3_AXIS_LEFT_V;
			gJoyX = PS3_X_ROT;
			gJoyY = PS3_Y_ROT;
			gJoyZ = PS3_Z_ROT;
			break; 
		default:
			printf("Unknown controller type for mapping\n");
	}
}

void print_joy_info() {
	printf("Name: %s\n", SDL_JoystickNameForIndex(0));
	printf("Number of Axes: %d\n", SDL_JoystickNumAxes(gJoystick));
	printf("Number of Buttons: %d\n", SDL_JoystickNumButtons(gJoystick));
	if(SDL_JoystickNumBalls(gJoystick) > 0) printf("Number of Balls: %d\n", SDL_JoystickNumBalls(gJoystick));
	if(strncmp(SDL_JoystickNameForIndex(0), "PLAYSTATION(R)3 Controller", 25) == 0) {
		// PS3 Rumble controller via BT
		gControllerType = PS3_CONTROLLER;
	}
	if(strncmp(SDL_JoystickNameForIndex(0), "Sony PLAYSTATION(R)3 Controller", 30) == 0) {
		// PS3 directly connected
		gControllerType = PS3_CONTROLLER;
	}
	map_joy();
}
#endif // !DISABLE_SDL

void *watch_input(void* arg)
{
	int c;
	struct termios orig_attr;
	struct termios new_attr;;

	// Set terminal to raw input
	tcgetattr(fileno(stdin), &orig_attr);
	memcpy(&new_attr, &orig_attr, sizeof(struct termios));
	new_attr.c_lflag &= ~(ECHO|ICANON);
	new_attr.c_cc[VTIME] = 0;
	new_attr.c_cc[VMIN] = 0;
	tcsetattr(fileno(stdin), TCSANOW, &new_attr);

	// What's a race condition?
	while (1) {
		c = getc(stdin);
		switch(c) {
			case 'w':
				throttle = 1;
				break;
			case 'a':
				turning = -1;
				break;
			case 's':
				throttle = -1;
				break;
			case 'd':
				turning = 1;
				break;
			case ' ':
				turning = 0;
				break;
			case 'q':
				lock_enabled = 1;
				unlock_enabled = 0;
				break;
			case 'e':
				lock_enabled = 0;
				unlock_enabled = 1;
				break;
			case '1':
				if (lock_enabled) {
					do_lock[0] = 1;
				} else if (unlock_enabled) {
					do_unlock[0] = 1;
				}
				break;
			case '2':
				if (lock_enabled) {
					do_lock[1] = 1;
				} else if (unlock_enabled) {
					do_unlock[1] = 1;
				}
				break;
			case '3':
				if (lock_enabled) {
					do_lock[2] = 1;
				} else if (unlock_enabled) {
					do_unlock[2] = 1;
				}
				break;
			case '4':
				if (lock_enabled) {
					do_lock[3] = 1;
				} else if (unlock_enabled) {
					do_unlock[3] = 1;
				}
				break;
		}
		usleep(5000);
	}
	// TODO: Restore terminal (this isn't reached)
	tcsetattr(fileno(stdin), TCSANOW, &orig_attr);
	return NULL;
}
void clear_screen()
{
	write(STDOUT_FILENO,  "\e[1;1H\e[2J", 10);
}

void redraw_tui() {
	clear_screen();
	printf("Indicator: ");
	if (turning) {
		printf("%s\n", turning < 0 ? "LEFT" : "RIGHT");
	} else {
		printf("OFF\n");
	}

	printf("Throttle: %s\n", throttle > 0 ? "ON" : "OFF");
	printf("Unlock enabled: %s\n", unlock_enabled ? "YES" : "NO");
	printf("Lock enabled: %s\n", lock_enabled ? "YES" : "NO");
}

// Plays background can traffic
void play_can_traffic() {
	char can2can[50];
	snprintf(can2can, 49, "%s=can0", ifr.ifr_name);
	if(execlp("canplayer", "canplayer", "-I", traffic_log, "-l", "i", can2can, NULL) == -1) {
		printf("WARNING: Could not execute canplayer. No bg data\n");
	}
}

void kill_child() {
	kill(play_pid, SIGINT);
}

void usage(char *msg) {
	if(msg) printf("%s\n", msg);
	printf("Usage: controls [options] <can>\n");
	printf("\t-s\tseed value from IC\n");
	printf("\t-l\tdifficulty level. 0-2 (default: %d)\n", DEFAULT_DIFFICULTY);
	printf("\t-T\ttraffic file to use for bg CAN traffic\n");
	printf("\t-t\ttext mode\n");
	printf("\t-k\tuse keyboard for input\n");
	printf("\t-X\tdisable background CAN traffic.  Cheating if doing RE but needed if playing on a real CANbus\n");
	printf("\t-d\tdebug mode\n");
	exit(1);
}

struct ui_t {
	void (*redraw)(void);
};

int main(int argc, char *argv[]) {
	int opt;
	struct sockaddr_can addr;
	int running = 1;
	int enable_canfd = 1;
	int enable_background_traffic = 1;
	struct stat st;
	struct ui_t ui = {0};

#if !(DISABLE_SDL)
	SDL_Event event;
	int button, axis; // Used for checking dynamic joystick mappings
	SDL_Surface *image = NULL;
	SDL_Window *window = NULL;
#endif // !DISABLE_SDL

	while ((opt = getopt(argc, argv, "Xdl:s:T:tkh?")) != -1) {
		switch(opt) {
			case 'l':
				difficulty = atoi(optarg);
				break;
			case 's':
				seed = atoi(optarg);
				break;
			case 'T':
				traffic_log = optarg;
				break;
			case 't':
				text_mode = 1;
				break;
			case 'k':
				keyboard_mode = 1;
				break;
			case 'd':
				debug = 1;
				break;
			case 'X':
				enable_background_traffic = 0;
				break;
			case 'h':
			case '?':
			default:
				usage(NULL);
				break;
		}
	}

#if DISABLE_SDL
	text_mode = 1;
	keyboard_mode = 1;
#endif // DISABLE_SDL

	if (optind >= argc) usage("You must specify at least one can device");

	if(stat(traffic_log, &st) == -1) {
		char msg[256];
		snprintf(msg, 255, "CAN Traffic file not found: %s\n", traffic_log);
		usage(msg);
	}

	/* open socket */
	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("socket");
		return 1;
	}

	addr.can_family = AF_CAN;

	strcpy(ifr.ifr_name, argv[optind]);
	if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
		perror("SIOCGIFINDEX");
		return 1;
	}
	addr.can_ifindex = ifr.ifr_ifindex;

	if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
				&enable_canfd, sizeof(enable_canfd))){
		printf("error when enabling CAN FD support\n");
		return 1;
	}

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}

	door_id = DEFAULT_DOOR_ID;
	signal_id = DEFAULT_SIGNAL_ID;
	speed_id = DEFAULT_SPEED_ID;

	if (seed) {
		srand(seed);
		door_id = (rand() % 2046) + 1;
		signal_id = (rand() % 2046) + 1;
		speed_id = (rand() % 2046) + 1;
		door_pos = rand() % 9;
		signal_pos = rand() % 9;
		speed_pos = rand() % 8;
		printf("Seed: %d\n", seed);
		door_len = door_pos + 1;
		signal_len = signal_pos + 1;
		speed_len = speed_len + 2;
	}

	if(difficulty > 0) {
		if (door_len < 8) {
			door_len += rand() % (8 - door_len);
		} else {
			door_len = 0;
		}
		if (signal_len < 8) {
			signal_len += rand() % (8 - signal_len);
		} else {
			signal_len = 0;
		}
		if (speed_len < 8) {
			speed_len += rand() % (8 - speed_len);
		} else {
			speed_len = 0;
		}
	}

	if(enable_background_traffic) {
		play_pid = fork();
		if((int)play_pid == -1) {
			printf("Error: Couldn't fork bg player\n");
			exit(-1);
		} else if (play_pid == 0) {
			play_can_traffic();
			// Shouldn't return
			exit(0);
		}
		atexit(kill_child);
	}

	// Input setup
	if (keyboard_mode) {
		// Spawn input thread
		pthread_t input_thread;
		if(pthread_create(&input_thread, NULL, watch_input, NULL)) {
			printf("Error: Couldn't create input thread\n");
		}
	} else {
#if !(DISABLE_SDL)
		if(SDL_Init(SDL_INIT_JOYSTICK) < 0 ) {
			printf("SDL Joystick subsystem could not be initialized\n");
			exit(40);
		}
		if(SDL_NumJoysticks() < 1) {
			printf(" Warning: No joysticks connected\n");
		} else {
			if(SDL_IsGameController(0)) {
				gGameController = SDL_GameControllerOpen(0);
				if(gGameController == NULL) {
					printf(" Warning: Unable to open game controller. %s\n", SDL_GetError() );
				} else {
					gJoystick = SDL_GameControllerGetJoystick(gGameController);
					print_joy_info();
				}
			} else {
				gJoystick = SDL_JoystickOpen(0);
				if(gJoystick == NULL) {
					printf(" Warning: Could not open joystick\n");
				} else {
					print_joy_info();
				}
			}
		}
#endif // !DISABLE_SDL
	}

	// UI Setup
	if (text_mode) {
		ui.redraw = redraw_tui;
	} else {
#if !(DISABLE_SDL)
		ui.redraw = redraw_gui;
		if(SDL_Init(SDL_INIT_VIDEO) < 0 ) {
			printf("SDL Video subsystem could not be initialized\n");
			exit(40);
		}
		window = SDL_CreateWindow("CANBus Control Panel", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
		if(window == NULL) {
			printf("Window could not be shown\n");
		}
		renderer = SDL_CreateRenderer(window, -1, 0);
		image = IMG_Load(get_data("joypad.png"));
		base_texture = SDL_CreateTextureFromSurface(renderer, image);
#endif // !DISABLE_SDL
	}
	ui.redraw();

	while(running) {
#if !(DISABLE_SDL)
		if (!keyboard_mode || !text_mode) {
			while(SDL_PollEvent(&event) != 0 ) {
				switch(event.type) {
					case SDL_QUIT:
						running = 0;
						break;
					case SDL_WINDOWEVENT:
						switch(event.window.event) {
							case SDL_WINDOWEVENT_ENTER:
							case SDL_WINDOWEVENT_RESIZED:
								ui.redraw();
								break;
						}
					case SDL_KEYDOWN:
						switch(event.key.keysym.sym) {
							case SDLK_UP:
								throttle = 1;
								break;
							case SDLK_LEFT:
								turning = -1;
								break;
							case SDLK_RIGHT:
								turning = 1;
								break;
							case SDLK_LSHIFT:
								lock_enabled = 1;
								if(unlock_enabled)
									do_unlock[0] = 1;
									do_unlock[1] = 1;
									do_unlock[3] = 1;
									do_unlock[4] = 1;
								break;
							case SDLK_RSHIFT:
								unlock_enabled = 1;
								if(lock_enabled)
									do_lock[0] = 1;
									do_lock[1] = 1;
									do_lock[3] = 1;
									do_lock[4] = 1;
								break;
							case SDLK_a:
								if(lock_enabled) {
									do_lock[0] = 1;
								} else if(unlock_enabled) {
									do_unlock[0] = 1;
								}
								break;
							case SDLK_b:
								if(lock_enabled) {
									do_lock[1] = 1;
								} else if(unlock_enabled) {
									do_unlock[1] = 1;
								}
								break;
							case SDLK_x:
								if(lock_enabled) {
									do_lock[2] = 1;
								} else if(unlock_enabled) {
									do_unlock[2] = 1;
								}
								break;
							case SDLK_y:
								if(lock_enabled) {
									do_lock[3] = 1;
								} else if(unlock_enabled) {
									do_unlock[3] = 1;
								}
								break;
						}
						kk_check(event.key.keysym.sym);
						break;
					case SDL_KEYUP:
						switch(event.key.keysym.sym) {
							case SDLK_UP:
								throttle = -1;
								break;
							case SDLK_LEFT:
							case SDLK_RIGHT:
								turning = 0;
								break;
							case SDLK_LSHIFT:
								lock_enabled = 0;
								break;
							case SDLK_RSHIFT:
								unlock_enabled = 0;
								break;
						}
						break;
					case SDL_JOYAXISMOTION:
						axis = event.jaxis.axis;
						if(axis == gAxisLeftH) {
							ud(event.jaxis.value);
						} else if(axis == gAxisLeftV) {
							turn(event.jaxis.value);
						} else if(axis == gAxisR2) {
							accelerate(event.jaxis.value);
						} else if(axis == gAxisRightH ||
								axis == gAxisRightV ||
								axis == gAxisL2 ||
								axis == gJoyX ||
								axis == gJoyY ||
								axis == gJoyZ) {
							// Do nothing, the axis is known just not connected
						} else {
							if (debug) printf("Unkown axis: %d\n", event.jaxis.axis);
						}
						break;
					case SDL_JOYBUTTONDOWN:
						button = event.jbutton.button;
						if(button == gButtonLock) {
							lock_enabled = 1;
							if(unlock_enabled)
								do_unlock[0] = 1;
								do_unlock[1] = 1;
								do_unlock[3] = 1;
								do_unlock[4] = 1;
						} else if(button == gButtonUnlock) {
							unlock_enabled = 1;
							if(lock_enabled)
								do_lock[0] = 1;
								do_lock[1] = 1;
								do_lock[3] = 1;
								do_lock[4] = 1;
						} else if(button == gButtonA) {
							if(lock_enabled) {
								do_lock[0] = 1;
							} else if(unlock_enabled) {
								do_unlock[0] = 1;
							}
							kk_check(SDLK_a);
						} else if (button == gButtonB) {
							if(lock_enabled) {
								do_lock[1] = 1;
							} else if(unlock_enabled) {
								do_unlock[2] = 1;
							}
							kk_check(SDLK_b);
						} else if (button == gButtonX) {
							if(lock_enabled) {
								do_lock[2] = 1;
							} else if(unlock_enabled) {
								do_unlock[2] = 1;
							}
							kk_check(SDLK_x);
						} else if (button == gButtonY) {
							if(lock_enabled) {
								do_lock[3] = 1;
							} else if(unlock_enabled) {
								do_unlock[3] = 1;
							}
							kk_check(SDLK_y);
						} else if (button == gButtonStart) {
							kk_check(SDLK_RETURN);
						} else {
							if(debug) printf("Unassigned button: %d\n", event.jbutton.button);
						}
						break;
					case SDL_JOYBUTTONUP:
						button = event.jbutton.button;
						if(button == gButtonLock) {
							lock_enabled = 0;
						} else if(button == gButtonUnlock) {
							unlock_enabled = 0;
						} else {
							//if(debug) printf("Unassigned button: %d\n", event.jbutton.button);
						}
						break;
					case SDL_JOYDEVICEADDED:
						// Only use the first controller
						if(event.cdevice.which == 0) {
							gJoystick = SDL_JoystickOpen(0);
							if(gJoystick) {
								print_joy_info();
							}
						}
						break;
					case SDL_JOYDEVICEREMOVED:
						if(event.cdevice.which == 0) {
							SDL_JoystickClose(gJoystick);
							gJoystick = NULL;
						}
						break;
					case SDL_CONTROLLERDEVICEADDED:
						// Only use the first controller
						if(gGameController == NULL) {
							gGameController = SDL_GameControllerOpen(0);
							gJoystick = SDL_GameControllerGetJoystick(gGameController);
							print_joy_info();
						}
						break;
					case SDL_CONTROLLERDEVICEREMOVED:
						if(event.cdevice.which == 0) {
							SDL_GameControllerClose(gGameController);
							gGameController = NULL;
						}
						break;
				}
			}
#endif // !DISABLE_SDL
			clock_gettime(CLOCK_MONOTONIC, &currentTime);
			current_ms = currentTime.tv_sec * 1000 + currentTime.tv_nsec / 1000000;
			check_accel();
			check_turn();
			check_locks();
			ui.redraw();
			usleep(5000);
		}

		close(s);
#if !(DISABLE_SDL)
		if (!keyboard_mode || !text_mode) {
			SDL_DestroyTexture(base_texture);
			SDL_FreeSurface(image);
			SDL_GameControllerClose(gGameController);
			SDL_DestroyRenderer(renderer);
			SDL_DestroyWindow(window);
			SDL_Quit();
		}
#endif // !DISABLE_SDL
	}
}
