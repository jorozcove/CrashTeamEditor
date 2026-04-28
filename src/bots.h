#pragma once

#include "geo.h"
#include "psx_types.h"
#include <filesystem>
#include <string>
#include <cstdint>
#include <vector>

// BAM (Binary Angle Measurement) conversion for rot[], which uses 256 units per full circle
// stored as int8, distinct from the FP_ONE fixed-point system used elsewhere.
static inline float BamToAngle(int8_t bam) { return (static_cast<float>(bam) * 360.0f) / 256.0f; }
static inline int8_t AngleToBam(float deg) { return static_cast<int8_t>(std::round((deg * 256.0f) / 360.0f)); }

class BotNode
{
public:

    BotNode() = default;
    BotNode(const PSX::NavFrame& frame);
  
    std::vector<uint8_t> Serialize() const;
    
    const Vec3& GetPos() const { return m_pos; }
    void        SetPos(const Vec3& pos) { m_pos = pos; }

    float GetPosX() const { return m_pos.x; }
    float GetPosY() const { return m_pos.y; }
    float GetPosZ() const { return m_pos.z; }
    void  SetPosX(float x) { m_pos.x = x; }
    void  SetPosY(float y) { m_pos.y = y; }
    void  SetPosZ(float z) { m_pos.z = z; }

    // Yaw: 0 = facing +Z, 90 = facing +X. Increases counter-clockwise (left turns).
    float GetYaw()   const { return m_yaw; }
    void  SetYaw(float deg) { m_yaw = deg; }

    // Pitch: positive = nose down (descending), negative = nose up (climbing).
    float GetPitch() const { return m_pitch; }
    void  SetPitch(float deg) { m_pitch = deg; }

    // Roll: lateral road banking.
    float GetRoll()  const { return m_roll; }
    void  SetRoll(float deg) { m_roll = deg; }

    uint16_t GetFlags()            const { return m_flags; }
    void     SetFlags(uint16_t f) { m_flags = f; }

    int16_t  GetPathChangeOpCode() const { return m_pathChangeOpCode; }
    void     SetPathChangeOpCode(int16_t v) { m_pathChangeOpCode = v; }

    uint8_t  GetGoBackCount()      const { return m_goBackCount; }
    void     SetGoBackCount(uint8_t v) { m_goBackCount = v; }

    uint8_t  GetSpecialBits()      const { return m_specialBits; }
    void     SetSpecialBits(uint8_t v) { m_specialBits = v; }

    int16_t  GetUnk2(int i)        const { return m_unk2[i]; }
    void     SetUnk2(int i, int16_t v) { m_unk2[i] = v; }

private:
    Vec3 m_pos = {};
    float m_yaw = 0.0f; // rot[1]
    float m_pitch = 0.0f; // rot[0]
    float m_roll = 0.0f; // rot[2]

    int16_t  m_unk2[2] = {};
    uint16_t m_flags = 0;
    int16_t  m_pathChangeOpCode = 0;
    uint8_t  m_goBackCount = 0;
    uint8_t  m_specialBits = 0;
};

class BotPath
{
public:
    BotPath() = default;

    BotPath(const PSX::NavHeader& header, const std::vector<PSX::NavFrame>& frames);
    void Clear();
    bool IsValid();

    std::vector<uint8_t> Serialize() const;

    const std::vector<BotNode>& GetNodes() const { return m_nodes; }
    std::vector<BotNode>& GetNodes() { return m_nodes; }

    size_t GetNodeCount() const { return m_nodes.size(); }

    const BotNode& GetNode(size_t index) const { return m_nodes.at(index); }
    BotNode& GetNode(size_t index) { return m_nodes.at(index); }

    void AddNode(const BotNode& node) { m_nodes.push_back(node); }
    void InsertNode(size_t index, const BotNode& node) { m_nodes.insert(m_nodes.begin() + index, node); }
    void RemoveNode(size_t index) { m_nodes.erase(m_nodes.begin() + index); }

    uint16_t GetMagic()        const { return m_magic; }
    void     SetMagic(uint16_t v) { m_magic = v; }

    uint32_t GetPosY()         const { return m_posY; }
    void     SetPosY(uint32_t v) { m_posY = v; }

    uint32_t GetOffLastPoint() const { return m_offLastPoint; }
    void     SetOffLastPoint(uint32_t v) { m_offLastPoint = v; }

    uint16_t GetPhysUnk(size_t i) const { return m_physUnk[i]; }
    void     SetPhysUnk(size_t i, uint16_t v) { m_physUnk[i] = v; }

private:
    std::vector<BotNode> m_nodes;
    uint16_t m_magic = 0;
    uint32_t m_posY = 0;
    uint32_t m_offLastPoint = 0;
    uint16_t m_physUnk[0x20] = {};
};