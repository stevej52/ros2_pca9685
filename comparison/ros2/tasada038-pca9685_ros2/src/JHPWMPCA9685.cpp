// MIT License

// Copyright (c) 2024 Takumi Asada
// Copyright (c) 2019 Pushkal Katara
// Copyright (c) 2015 Jetsonhacks

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/*------------------------------------------------------------------------------*/
/* Include */
/*------------------------------------------------------------------------------*/
#include <pca9685_ros2/JHPWMPCA9685.h>
#include <math.h>

/*------------------------------------------------------------------------------*/
/* Constructor */
/*------------------------------------------------------------------------------*/
PCA9685::PCA9685(int address, int i2c_bus) {
    kI2CBus = i2c_bus;           // Default I2C bus for Jetson TK1
    kI2CAddress = address ;      // Defaults to 0x40 for PCA9685 ; jumper settable
    error = 0 ;
}

/*------------------------------------------------------------------------------*/
/* Destructor */
/*------------------------------------------------------------------------------*/
PCA9685::~PCA9685() {
    closePCA9685() ;
}

/*------------------------------------------------------------------------------*/
/* Function */
/*------------------------------------------------------------------------------*/
bool PCA9685::openPCA9685()
{
    char fileNameBuffer[32];
    sprintf(fileNameBuffer,"/dev/i2c-%d", kI2CBus);
    kI2CFileDescriptor = open(fileNameBuffer, O_RDWR);
    if (kI2CFileDescriptor < 0) {
        // Could not open the file
       error = errno ;
       return false ;
    }
    if (ioctl(kI2CFileDescriptor, I2C_SLAVE, kI2CAddress) < 0) {
        // Could not open the device on the bus
        error = errno ;
        return false ;
    }
    return true ;
}

void PCA9685::closePCA9685()
{
    if (kI2CFileDescriptor > 0) {
        close(kI2CFileDescriptor);
        // WARNING - This is not quite right, need to check for error first
        kI2CFileDescriptor = -1 ;
    }
}

void PCA9685::reset () {
    writeByte(PCA9685_MODE1, PCA9685_ALLCALL );
    writeByte(PCA9685_MODE2, PCA9685_OUTDRV) ;
    // Wait for oscillator to stabilize
    usleep(5000) ;
}

// Sets the frequency of the PWM signal
// Frequency is ranged between 40 and 1000 Hertz
void PCA9685::setPWMFrequency ( float frequency ) {
    printf("Setting PCA9685 PWM frequency to %f Hz\n",frequency) ;
    float rangedFrequency = fmin(fmax(frequency,40),1000) ;
    int prescale = (int)(25000000.0f / (4096 * rangedFrequency) - 0.5f) ;
    // For debugging
    // printf("PCA9685 Prescale: 0x%02X\n",prescale) ;
    int oldMode = readByte(PCA9685_MODE1) ;
     int newMode = ( oldMode & 0x7F ) | PCA9685_SLEEP ;
    writeByte(PCA9685_MODE1, newMode) ;
    writeByte(PCA9685_PRE_SCALE, prescale) ;
    writeByte(PCA9685_MODE1, oldMode) ;
    // Wait for oscillator to stabilize
    usleep(5000) ;
    writeByte(PCA9685_MODE1, oldMode | PCA9685_RESTART) ;
}

// Channels 0-15
// Channels are in sets of 4 bytes
void PCA9685::setPWM ( int channel, int onValue, int offValue) {
    writeByte(PCA9685_LED0_ON_L+4*channel, onValue & 0xFF) ;
    writeByte(PCA9685_LED0_ON_H+4*channel, onValue >> 8) ;
    writeByte(PCA9685_LED0_OFF_L+4*channel, offValue & 0xFF) ;
    writeByte(PCA9685_LED0_OFF_H+4*channel, offValue >> 8) ;
}

void PCA9685::setAllPWM (int onValue, int offValue) {
    writeByte(PCA9685_ALL_LED_ON_L, onValue & 0xFF) ;
    writeByte(PCA9685_ALL_LED_ON_H, onValue >> 8) ;
    writeByte(PCA9685_ALL_LED_OFF_L, offValue & 0xFF) ;
    writeByte(PCA9685_ALL_LED_OFF_H, offValue >> 8) ;
}

// Read the given register
int PCA9685::readByte(int readRegister)
{
    int toReturn = i2c_smbus_read_byte_data(kI2CFileDescriptor, readRegister);
    if (toReturn < 0) {
        printf("PCA9685 Read Byte error: %d",errno) ;
        error = errno ;
        toReturn = -1 ;
    }
    // For debugging
    // printf("Device 0x%02X returned 0x%02X from register 0x%02X\n", kI2CAddress, toReturn, readRegister);
    return toReturn ;
}

// Write the the given value to the given register
int PCA9685::writeByte(int writeRegister, int writeValue)
{   // For debugging:
    // printf("Wrote: 0x%02X to register 0x%02X \n",writeValue, writeRegister) ;
    int toReturn = i2c_smbus_write_byte_data(kI2CFileDescriptor, writeRegister, writeValue);
    if (toReturn < 0) {
        printf("PCA9685 Write Byte error: %d",errno) ;
        error = errno ;
        toReturn = -1 ;
    }
    return toReturn ;
}