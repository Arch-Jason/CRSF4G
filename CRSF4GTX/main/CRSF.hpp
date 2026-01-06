#pragma once

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <string>
#include "udp.hpp"

// ---------------- Configuration ----------------
#define MAX_PARAM_COUNT 10      // 最大支持参数数量
#define MAX_PAYLOAD_SIZE 64     // CRSF 物理帧最大长度
#define MAX_CHUNK_SIZE 58       // 单个包能承载的最大有效数据 (64 - Sync - Len - Type - Dest - Orig - P# - ChunkRem - CRC)

// ---------------- Protocol Constants ----------------
#define CRSF_SYNC_BYTE          0xC8 // to handset when pong
#define CRSF_ADDRESS_BROADCAST  0x00
#define CRSF_ADDRESS_RADIO      0xEA
#define CRSF_ADDRESS_MODULE     0xEE

#define CRSF_TYPE_GPS           0x02
#define CRSF_TYPE_PING          0x28
#define CRSF_TYPE_DEVICE_INFO   0x29
#define CRSF_TYPE_PARAM_ENTRY   0x2B
#define CRSF_TYPE_PARAM_READ    0x2C
#define CRSF_TYPE_PARAM_WRITE   0x2D

#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16

// Parameter Types
enum ParamType : uint8_t {
    PTV_FLOAT           = 8,
    PTV_TEXT_SELECTION  = 9,
    PTV_STRING          = 10,
    PTV_FOLDER          = 11,
    PTV_INFO            = 12,
    PTV_COMMAND         = 13
};

// ---------------- Helper Macros ----------------
#define PACKED __attribute__((packed))
// ARM Cortex-M 内置指令，快速大小端转换
#define TO_BE32(x) __builtin_bswap32(x) 
#define TO_BE16(x) __builtin_bswap16(x)

// ---------------- Abstract Base Class ----------------
class CRSF_Param {
public:
    uint8_t id;
    uint8_t parentId;
    const char* name;
    ParamType type;

    CRSF_Param(uint8_t _id, uint8_t _parentId, const char* _name, ParamType _type)
        : id(_id), parentId(_parentId), name(_name), type(_type) {}

    virtual ~CRSF_Param() {}

    // 核心虚函数：将参数的完整定义序列化到 buffer 中
    // 返回写入的总字节数
    virtual uint16_t Serialize(uint8_t* buffer) = 0;

    // 处理写入请求 (返回 true 表示值已更新)
    virtual bool UpdateValue(const uint8_t* data, uint8_t len) { return false; }
};

// ---------------- Concrete Classes ----------------

// 1. Folder (文件夹)
class CRSF_ParamFolder : public CRSF_Param {
public:
    // List of children IDs, terminated by 0xFF. 
    // Simplified: we won't store children list dynamically here to save RAM, 
    // usually we just construct it on the fly or pass a static array.
    const uint8_t* childrenIDs; 
    uint8_t childrenCount;

    CRSF_ParamFolder(uint8_t _id, uint8_t _parentId, const char* _name, const uint8_t* _children, uint8_t _count)
        : CRSF_Param(_id, _parentId, _name, PTV_FOLDER), childrenIDs(_children), childrenCount(_count) {}

    uint16_t Serialize(uint8_t* buf) override;
};

// 2. Info (只读文本)
class CRSF_ParamInfo : public CRSF_Param {
public:
    const char* valueStr;

    CRSF_ParamInfo(uint8_t _id, uint8_t _parentId, const char* _name, const char* _val)
        : CRSF_Param(_id, _parentId, _name, PTV_INFO), valueStr(_val) {}

    uint16_t Serialize(uint8_t* buf) override;
};

// 3. Float (数字)
class CRSF_ParamFloat : public CRSF_Param {
public:
    int32_t value;
    int32_t min;
    int32_t max;
    int32_t def;
    uint8_t decimal_point;
    int32_t step;
    const char* unit;

    CRSF_ParamFloat(uint8_t _id, uint8_t _parentId, const char* _name, 
                    int32_t _val, int32_t _min, int32_t _max, uint8_t _decimal_point, int32_t _step, const char* _unit)
        : CRSF_Param(_id, _parentId, _name, PTV_FLOAT), 
          value(_val), min(_min), max(_max), def(_val), decimal_point(_decimal_point), step(_step), unit(_unit) {}

    uint16_t Serialize(uint8_t* buf) override;
    bool UpdateValue(const uint8_t* data, uint8_t len) override;
};

// 4. Text Selection (选项列表)
class CRSF_ParamSelection : public CRSF_Param {
public:
    uint8_t value; // Index
    uint8_t max_options;
    const char* options; // Semicolon separated: "OptA;OptB;OptC"
    const char* unit;

    CRSF_ParamSelection(uint8_t _id, uint8_t _parentId, const char* _name, 
                        uint8_t _val, const char* _options, const char* _unit)
        : CRSF_Param(_id, _parentId, _name, PTV_TEXT_SELECTION), 
          value(_val), options(_options), unit(_unit) {
              std::string options_str(_options);
              max_options = std::count(options_str.begin(), options_str.end(), ';');
          }

    uint16_t Serialize(uint8_t* buf) override;
    bool UpdateValue(const uint8_t* data, uint8_t len) override;
};

// 5. String
class CRSF_ParamString : public CRSF_Param {
public:
    char* value;
    uint8_t string_max_len;

    CRSF_ParamString(uint8_t _id, uint8_t _parentId, const char* _name, 
                        const char* _val, uint8_t _string_max_len)
        : CRSF_Param(_id, _parentId, _name, PTV_STRING), 
          value((char*) _val), string_max_len(_string_max_len) {
          }

    uint16_t Serialize(uint8_t* buf) override;
    bool UpdateValue(const uint8_t* data, uint8_t len) override;
};

// ---------------- Main Module Class ----------------
class CRSF_TxModule {
private:
    CRSF_Param* params[MAX_PARAM_COUNT];
    uint8_t paramCount;
    int16_t channels[16];
    
    // Hardware abstractions
    void (*txFunc)(uint8_t*, uint16_t);

    // Helpers
    uint8_t CalcCRC8(const uint8_t* data, uint8_t len);
    void SendRawFrame(uint8_t type, uint8_t* payload, uint8_t len);
    void SendDeviceInfo();
    void SendParamChunk(uint8_t paramIdx, uint8_t chunkIdx);

public:
    CRSF_TxModule(void (*_txFunc)(uint8_t*, uint16_t));
    void AddParam(CRSF_Param* param);
    CRSF_Param* GetParam(const char* name);
    void ProcessPacket(uint8_t* buf, uint8_t len);
    void ParseChannels(const uint8_t *payload);
};