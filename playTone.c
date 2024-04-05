
#define uint16_t int

void playTone(uint16_t freq) {
	uint16_t arr = 8000000 / (3 * freq);
	TIM3->ARR = arr;
	TIM3->CCR2 = arr /2;
}



void playTune(uint16_t* frequencies, uint16_t* durations, uint16_t length) {
	for (int i = 0; i < length; i++) {
		if (frequencies[i] == 0) {
			disableTIM3();
		}
		else {
			enableTIM3();
			playTone(frequencies[i]);
		}

		HAL_Delay(durations[i]);
	}

	disableTIM3();
}

int main() {

	//
	uint16_t[3] frequencies = {261, 329, 392, 523};
	uint16_t[3] durations = {200, 200, 200, 200};

	// shave and a haircut
	uint16_t[3] frequencies = {100, 0, 100, 0, 100, 0, 100, 0, 100, 0};
	uint16_t[3] durations = {200, 400, 100, 200, 100, 200, 200, 400, 200, 400};

	playTune(frequencies, durations, 3);

}


