readme.txt
----------
This project is a port of the LoRaWAN Demo using GNU gcc and Eclipse.

With logging enabled (NRF_LOG_ENABLED in sdk_config.h), it logs to SEGGER RTT.


Decoder:
 function Decoder(bytes, port) {
    // Decode an uplink message from a buffer
    // (array) of bytes to an object of fields.
    var decoded = {};

    decoded.latitude = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
    decoded.latitude = (decoded.latitude / 16777215.0 * 180) - 90;

    decoded.longitude = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
    decoded.longitude = (decoded.longitude / 16777215.0 * 360) - 180;

    var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
    var sign = bytes[6] & (1 << 7);
    if(sign) {
        decoded.altitude = 0xFFFF0000 | altValue;
    } else {
        decoded.altitude = altValue;
    }
    decoded.hdop = bytes[8] / 10.0;
    return decoded;
}

7FFFFF7FFFFF00000A