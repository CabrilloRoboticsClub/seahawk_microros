#include <stdio.h>
#include "pico/stdlib.h"
#include "dshot.h"

struct dshot_controller controller0;

int main(void)
{
	int time;

	stdio_init_all();

	/* initialize controller 0 with DSHOT600 on pio0, state machine 0
	 * with 4 channels starting on pin 2 */
	dshot_controller_init(&controller0, 300, pio0, 0, 2, 4);
	

	while (true) {
		time = to_ms_since_boot(get_absolute_time()) % 4000;
		/* at the beginning of the first second, spin the first motor. On the second second, spin the second motor, etc */
		for (int i = 0; i < 4; i++) {
			if (time > (i * 1000) && time < (i * 1000 + 100))
				dshot_throttle(&controller0, i, 100);
			else
				dshot_throttle(&controller0, i, 0);
		}

		/* run the dshot loop */
		dshot_loop(&controller0);
	}

	return 0;
}
