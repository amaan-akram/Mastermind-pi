#include <stdio.h> 
#include <stdlib.h> 
#include <string.h>

// Constants representing the GPIO pins of the LEDs.
#define RED_LED 5
#define YELLOW_LED 13

int *getInput() {
	int i, j; 
	static int code[3];
	for (i=0; i<3; i++) {
		char message[16];
		sprintf(message, "Enter Digit %d",i+1);
		say(message,""); 
		code[i] = clickCount();
		if (code[i] == 0) { 
			code[i] = 1; 
		} else if (code[i] > 3) { 
			code[i] = 3; 
		}
		wink(RED_LED); 
		say("Confirming...","");
		for (j=0; j<code[i]; j++) { 
			wink(YELLOW_LED); 
		}
	}
	say("Code Recieved","");
	wink(RED_LED);
	wink(RED_LED);
	return code;
} 

int *getGuess() {
	int i, j; 
	int static guess[3]; 
	char message[16];
	for (i=0; i<3; i++) {
		sprintf(message, "Enter Digit %d",i+1);
		say(message,""); 
		guess[i] = clickCount(); 
		if(guess[i] == 0) {
		    guess[i] = 1;
		} else if (guess[i] > 3) {
		    guess[i] = 3;
		}
		wink(RED_LED); 
		for (j=0; j<guess[i]; j++) { 
			wink(YELLOW_LED); 
		} 
	}
	say("Code Recieved","");
	wink(RED_LED); 
	wink(RED_LED); 
	return guess; 
}

int compareCode(int *guess, int *code) { 
	int trueCount =0;
	int i; 
	for (i=0; i<3; i++) {
		if (guess[i] == code[i]) {
			trueCount++; 
		} 
	}
	return trueCount; 
}

int closeCode(int *guess, int *code) {
	int closeCount = 0; 
	int i,j; 
	for (j=0; j<3; j++) {
		if (guess[j] != code[j]) {
			for (i=0; i<3; i++) {
				if ((guess[j]==code[i] && (i!=j) && (code[i] != guess[i]))) {
					closeCount++; 
					break;
				} 
			} 
		} 
	}
	return closeCount;  
}


int main(int argc, char *argv[]) {
	int debug; 
	if (argv[1] == NULL) { 
		debug = 0; 
	} else if (strcmp(argv[1],"-d") == 0) { 
		debug = 1; 
		printf("Mastermind RPi2: Debug Mode\n---------------------------");
	} else {
		fprintf(stderr, "Setup: Unknown command '%s'.\nDid you mean '-d'?\n",argv[1]); 
		exit(0);
	}
	setup();
	fflush(stdout);
	say("Mastermind","RPi2 Rendering");
	sleep(2);
	say("Code Breaker...","Look Away!"); 
	sleep(2);
	say("Code Maker...","Get Ready!");
	sleep(2);
	int *code; 
	code = (int*)malloc(3*sizeof(int));
	code = getInput();
	if (debug) { 
		printf("\nCode Maker's Secret: %d%d%d\n", code[0],code[1],code[2]);
	} 
	sleep(1);
	int *guess; 
	int guessCount = 0;
	guess = (int*)malloc(3*sizeof(int));  
	int correct = 0;
	while (correct != 3) {
		say("Code Breaker...","Get Ready!"); 
	    sleep(1);
	    guessCount++;
		guess = getGuess(); 
		if (debug) {
			printf("\nAttempt %d\n---------\nCode Breaker's Guess: %d%d%d\n",guessCount, guess[0],guess[1],guess[2]);  
		}
		correct = compareCode(guess,code); 
		if (correct < 3) { 
			int close = closeCode(guess,code);
			checkLED(correct, close);
			char correct_message[16]; 
			sprintf(correct_message,"Correct: %d",correct); 
			char close_message[16]; 
			sprintf(close_message,"Close: %d",close);
			printf("Correct: %d\n",correct); 
			printf("Close: %d\n",close);  
			say(close_message,correct_message);
			sleep(3);
		} else {
			printf("Code Breaker has won.\n");
			char attemptMessage[16]; 
			sprintf(attemptMessage,"Attempts: %d",guessCount); 
			say("SUCCESS!",attemptMessage);
			exit(0);
		}   
	}
	return 0; 
}
