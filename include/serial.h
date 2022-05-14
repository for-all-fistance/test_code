#include <wiringPi.h>
int communicate(char const *message);
char serial_get();
PI_THREAD (serial_thread,std::ref(command));

