#include "CRSF.hpp"
#include <cstdint>

volatile uint64_t current_latency_ms;

// ---------------- CRC Implementation ----------------
static const uint8_t crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

// ---------------- Buffer Writers (Big Endian) ----------------
static void WriteBufString(uint8_t*& buf, const char* str) {
    if (!str) { *buf++ = 0; return; }
    while (*str) *buf++ = *str++;
    *buf++ = 0; // Null terminator
}

static void WriteBufU32(uint8_t*& buf, int32_t val) {
    uint32_t v = TO_BE32((uint32_t)val);
    memcpy(buf, &v, 4);
    buf += 4;
}

// ---------------- Parameter Serializers ----------------

// Folder
uint16_t CRSF_ParamFolder::Serialize(uint8_t* buf) {
    uint8_t* start = buf;
    *buf++ = parentId;
    *buf++ = (uint8_t)type; // 0x0B
    WriteBufString(buf, name);
    // List of children
    for(int i=0; i<childrenCount; i++) {
        *buf++ = childrenIDs[i];
    }
    *buf++ = 0xFF; // End marker
    return buf - start;
}

// Info
uint16_t CRSF_ParamInfo::Serialize(uint8_t* buf) {
    uint8_t* start = buf;
    *buf++ = parentId;
    *buf++ = (uint8_t)type; // 0x0C
    WriteBufString(buf, name);
    WriteBufString(buf, valueStr);
    return buf - start;
}

// Float
uint16_t CRSF_ParamFloat::Serialize(uint8_t* buf) {
    uint8_t* start = buf;
    *buf++ = parentId;
    *buf++ = (uint8_t)type; // 0x08
    WriteBufString(buf, name);
    WriteBufU32(buf, value);
    WriteBufU32(buf, min);
    WriteBufU32(buf, max);
    WriteBufU32(buf, def);
    *buf++ = decimal_point;
    WriteBufU32(buf, step);
    WriteBufString(buf, unit);
    return buf - start;
}

bool CRSF_ParamFloat::UpdateValue(const uint8_t* data, uint8_t len) {
    if (len < 4) return false;
    // Data comes as Big Endian int32
    uint32_t raw;
    memcpy(&raw, data, 4);
    value = (int32_t)TO_BE32(raw); // Convert BE to LE (Host)
    return true;
}

// Text Selection
uint16_t CRSF_ParamSelection::Serialize(uint8_t* buf) {
    uint8_t* start = buf;
    *buf++ = parentId;
    *buf++ = (uint8_t)type; // 0x09
    WriteBufString(buf, name);
    WriteBufString(buf, options);
    *buf++ = value;
    *buf++ = 0; // Min index
    *buf++ = max_options; // Max index
    *buf++ = 0; // Default
    WriteBufString(buf, unit);
    return buf - start;
}

bool CRSF_ParamSelection::UpdateValue(const uint8_t* data, uint8_t len) {
    if (len < 1) return false;
    value = data[0];
    return true;
}

uint16_t CRSF_ParamString::Serialize(uint8_t* buf) {
    uint8_t* start = buf;
    *buf++ = parentId;
    *buf++ = (uint8_t)type;
    WriteBufString(buf, name);
    WriteBufString(buf, value);
    *buf++ = string_max_len;
    return buf - start;
}

bool CRSF_ParamString::UpdateValue(const uint8_t* data, uint8_t len) {
    // 安全检查：防止溢出
    if (len >= string_max_len)
        len = string_max_len - 1;
    else
        len -=  1;
    memcpy(value, data, len);
    value[len] = '\0'; 
    return true;
}

// ---------------- TX Module Implementation ----------------

CRSF_TxModule::CRSF_TxModule(void (*_txFunc)(uint8_t*, uint16_t)) {
    txFunc = _txFunc;
    paramCount = 0;
    // Clear params array
    for(int i=0; i<MAX_PARAM_COUNT; i++) params[i] = nullptr;
}

void CRSF_TxModule::AddParam(CRSF_Param* param) {
    if (paramCount < MAX_PARAM_COUNT) {
        param->id = paramCount;
        params[paramCount++] = param;
    }
}

CRSF_Param* CRSF_TxModule::GetParam(const char* name) {
    for (CRSF_Param* &param : params) {
        if (param->name == name)
        return param;
    }
    return NULL;
}

uint8_t CRSF_TxModule::CalcCRC8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc = crc8tab[crc ^ data[i]];
    }
    return crc;
}

// 通用的帧发送函数 (自动加头、加尾)
// type: 帧类型
// payload: 纯载荷 (不含 Sync, Len, Type, CRC)
void CRSF_TxModule::SendRawFrame(uint8_t type, uint8_t* payload, uint8_t len) {
    uint8_t buffer[MAX_PAYLOAD_SIZE + 10];
    uint8_t pos = 0;

    buffer[pos++] = CRSF_SYNC_BYTE; // Sync Byte
    buffer[pos++] = len + 2;             // Length = PayloadLen + Type(1) + CRC(1)
    buffer[pos++] = type;                // Type

    memcpy(&buffer[pos], payload, len);  // Payload
    pos += len;

    uint8_t crc = CalcCRC8(&buffer[2], buffer[1] - 1);
    buffer[pos++] = crc;

    if (txFunc) txFunc(buffer, pos);
}

// 发送设备信息 (0x29)
void CRSF_TxModule::SendDeviceInfo() {
    uint8_t buf[60];
    uint8_t pos = 0;
    
    buf[pos++] = CRSF_ADDRESS_RADIO; // Destination: Radio
    buf[pos++] = CRSF_ADDRESS_MODULE; // Origin: Me

    const char* name = "CRSF4G TX";
    int nameLen = strlen(name) + 1;
    memcpy(&buf[pos], name, nameLen); pos += nameLen;

    uint32_t sn = TO_BE32(0); memcpy(&buf[pos], &sn, 4); pos += 4;
    uint32_t hw = TO_BE32(0); memcpy(&buf[pos], &hw, 4); pos += 4;
    uint32_t fw = TO_BE32(0); memcpy(&buf[pos], &fw, 4); pos += 4;
    
    buf[pos++] = paramCount; // Total Params
    buf[pos++] = 0;          // Param Version

    SendRawFrame(CRSF_TYPE_DEVICE_INFO, buf, pos);
}

// 核心：处理参数分片发送 (Chunking)
void CRSF_TxModule::SendParamChunk(uint8_t paramIdx, uint8_t chunkIdx) {
    // 1. 找到参数
    CRSF_Param* p = nullptr;
    for(int i=0; i<paramCount; i++) {
        if (params[i]->id == paramIdx) {
            p = params[i];
            break;
        }
    }

    if (!p) return; // Param not found

    // 2. 将整个参数定义序列化到一个大Buffer中 (栈内存需足够，或者用静态buffer)
    uint8_t fullDef[128]; 
    uint16_t totalLen = p->Serialize(fullDef);

    // 3. 计算切片位置
    uint16_t offset = chunkIdx * MAX_CHUNK_SIZE;
    uint8_t chunksRemaining = 0;

    if (offset >= totalLen) {
        // 请求越界，发送空包或剩余0
        return; 
    }

    uint8_t lenToSend = 0;
    if (totalLen - offset > MAX_CHUNK_SIZE) {
        lenToSend = MAX_CHUNK_SIZE;
        // 计算还剩多少块
        uint16_t remainingBytes = totalLen - (offset + lenToSend);
        chunksRemaining = (remainingBytes + MAX_CHUNK_SIZE - 1) / MAX_CHUNK_SIZE;
    } else {
        lenToSend = totalLen - offset;
        chunksRemaining = 0;
    }

    // 4. 组装 0x2B 帧的 Payload
    // Format: [Dest][Orig][ParamID][ChunksRem][Payload Chunk...]
    uint8_t frameLoad[64];
    uint8_t pos = 0;

    frameLoad[pos++] = CRSF_ADDRESS_RADIO;
    frameLoad[pos++] = CRSF_ADDRESS_MODULE;
    frameLoad[pos++] = p->id;
    frameLoad[pos++] = chunksRemaining;

    memcpy(&frameLoad[pos], &fullDef[offset], lenToSend);
    pos += lenToSend;

    SendRawFrame(CRSF_TYPE_PARAM_ENTRY, frameLoad, pos);
}

// 路由器
void CRSF_TxModule::ProcessPacket(uint8_t* buf, uint8_t len) {
    const char* TAG = "CRSF";
    if (len < 4) return;
    if (len >= sizeof(time_packet_t)) {
        // 遍历 buffer
        for (int i = 0; i <= len - (int)sizeof(time_packet_t); i++) {
            // 1. 先检查 Magic
            if (buf[i] == TIME_SYNC_MAGIC) { 
                // 2. 再次检查 Type
                if (buf[i+1] == PACKET_TYPE_PONG) { // PACKET_TYPE_PONG
                    time_packet_t temp_pkt;
                    memcpy(&temp_pkt, &buf[i], sizeof(time_packet_t));
                    handle_incoming_pong(&temp_pkt);
                    i += (sizeof(time_packet_t) - 1);
                }
            }
        }
    }
    
    uint8_t type = buf[2];

    switch (type) {
        case CRSF_TYPE_PING: // 0x28
            // Check destination (Broadcast 0x00 or Me 0xEE)
            if (buf[3] == 0x00 || buf[3] == CRSF_ADDRESS_MODULE) {
                SendDeviceInfo();
            }
            break;

        case CRSF_TYPE_PARAM_READ: // 0x2C
            // Payload: [Dest][Orig][ParamIdx][ChunkIdx]
            if (buf[3] == CRSF_ADDRESS_MODULE) { // Is it for me?
                uint8_t pIdx = buf[5];
                uint8_t cIdx = buf[6];
                SendParamChunk(pIdx, cIdx);
            }
            break;
            
        case CRSF_TYPE_PARAM_WRITE: // 0x2D
            // Payload: [Dest][Orig][ParamIdx][Value...]
            if (buf[3] == CRSF_ADDRESS_MODULE) {
                uint8_t pIdx = buf[5];
                CRSF_Param* p = nullptr;
                for(int i=0; i<paramCount; i++) {
                    if (params[i]->id == pIdx) {
                        p = params[i];
                        break;
                    }
                }
                if (p) {
                    // Value starts at buf[6]
                    if (p->UpdateValue(&buf[6], len - 6 - 1)) { // -1 for CRC
                        // 写入成功后，通常需要回传一次 0x2B 确认（特别是对于 Command 类型），或简单回传值
                        // 为了简单，这里回传 Chunk 0 刷新界面
                        SendParamChunk(pIdx, 0);
                    }
                }
            }
            break;

        default:
            crsf_udp_send(buf, len);
    }
}

void CRSF_TxModule::ParseChannels(const uint8_t* payload)
{
    uint32_t bitbuf = 0;
    uint8_t bitcount = 0;
    uint8_t idx = 0;

    for (int i = 0; i < 16; i++) {
        while (bitcount < 11) {
            bitbuf |= ((uint32_t)payload[idx++]) << bitcount;
            bitcount += 8;
        }

        channels[i] = bitbuf & 0x7FF; // 11 bit
        bitbuf >>= 11;
        bitcount -= 11;
    }
}

// Get time in microseconds
int64_t get_time() {
    return esp_timer_get_time();
}

// Received a packet from LTE that starts with 0xFA
void CRSF_TxModule::handle_incoming_pong(time_packet_t* pkt) {
    if (pkt->magic == TIME_SYNC_MAGIC && pkt->type == PACKET_TYPE_PONG) {
        int64_t now = esp_timer_get_time();
        int64_t rtt = now - pkt->timestamp; 
        
        // 过滤异常数据：如果时间差是负数或者大得离谱，说明数据错乱或者重启了
        if (rtt > 0 && rtt < 5000000) {
            uint32_t delay_ms = (uint32_t)(rtt / 1000); 
            current_latency_ms = delay_ms;
            // printf("Ping Success! RTT: %u ms\n", delay_ms);
        }
    }
}

void CRSF_TxModule::SendPing(void) {
    current_latency_ms = 0;

    time_packet_t pkt;
    pkt.magic = TIME_SYNC_MAGIC;
    pkt.type = PACKET_TYPE_PING;
    pkt.timestamp = get_time();
    
    // Send to LTE/UDP bridge (UART_NUM_1 in your code)
    crsf_udp_send((const uint8_t*)&pkt, sizeof(pkt));
}