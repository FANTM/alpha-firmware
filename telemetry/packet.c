#include "packet.h"

#include "fantmBLE.h"

#include "app_util_platform.h"

#include "nordic_common.h"

#include "boards.h"

#include "app_error.h"

static void serializePacket(Packet_t *packet, uint8_t *data);
static void serializePacket(Packet_t *packet, uint8_t *data) {
data[0] = 0xff & (packet->accelX >> 8);
data[1] = 0xff & (packet->accelX >> 0);
data[2] = 0xff & (packet->accelY >> 8);
data[3] = 0xff & (packet->accelY >> 0);
data[4] = 0xff & (packet->accelZ >> 8);
data[5] = 0xff & (packet->accelZ >> 0);
data[6] = 0xff & (packet->gyroX >> 8);
data[7] = 0xff & (packet->gyroX >> 0);
data[8] = 0xff & (packet->gyroY >> 8);
data[9] = 0xff & (packet->gyroY >> 0);
data[10] = 0xff & (packet->gyroZ >> 8);
data[11] = 0xff & (packet->gyroZ >> 0);
data[12] = 0xff & (packet->magX >> 8);
data[13] = 0xff & (packet->magX >> 0);
data[14] = 0xff & (packet->magY >> 8);
data[15] = 0xff & (packet->magY >> 0);
data[16] = 0xff & (packet->magZ >> 8);
data[17] = 0xff & (packet->magZ >> 0);
data[18] = 0xff & (packet->temp >> 8);
data[19] = 0xff & (packet->temp >> 0);

}

void telemetrySend(Packet_t *packet) {

    uint16_t size=PACKET_SIZE;
    uint8_t data[PACKET_SIZE];
    serializePacket(packet, data);
    writeBLE(data, &size);
}
