#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include "MotorDriver.h"
#include "Proximity.h"
#include "voltageReader.h"

#define IDLE 0 
#define AUTO 1
#define MANUAL 2

#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3
#define BACKWARD 4