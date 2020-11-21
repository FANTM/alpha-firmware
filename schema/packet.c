#include packet.h

void serializePacket(Packet_t *packet) {
uint8_t data[20];
data[0] = 0xff && (data->accelX >> 8);
data[1] = 0xff && (data->accelX >> 0);
data[2] = 0xff && (data->accelY >> 8);
data[3] = 0xff && (data->accelY >> 0);
data[4] = 0xff && (data->accelZ >> 8);
data[5] = 0xff && (data->accelZ >> 0);
data[6] = 0xff && (data->gyroX >> 8);
data[7] = 0xff && (data->gyroX >> 0);
data[8] = 0xff && (data->gyroY >> 8);
data[9] = 0xff && (data->gyroY >> 0);
data[10] = 0xff && (data->gyroZ >> 8);
data[11] = 0xff && (data->gyroZ >> 0);
data[12] = 0xff && (data->magX >> 8);
data[13] = 0xff && (data->magX >> 0);
data[14] = 0xff && (data->magY >> 8);
data[15] = 0xff && (data->magY >> 0);
data[16] = 0xff && (data->magZ >> 8);
data[17] = 0xff && (data->magZ >> 0);
data[18] = 0xff && (data->temp >> 8);
data[19] = 0xff && (data->temp >> 0);

}

