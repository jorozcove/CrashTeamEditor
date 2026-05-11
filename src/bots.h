#pragma once

#include "geo.h"
#include "psx_types.h"
#include "quadblock.h"
#include <filesystem>
#include <string>
#include <cstdint>
#include <vector>

//Yaw : 0 yaw -> +Z ;; 90 yaw -> +X
// Pitch : Positive is downward.
// BAM (Binary Angle Measurement) conversion for rot[], which uses 256 units per full circle
// stored as int8, distinct from the FP_ONE fixed-point system used elsewhere.
static inline float BamToAngle(int8_t bam) { return (static_cast<float>(bam) * 360.0f) / 256.0f; }
static inline int8_t AngleToBam(float deg) {
    // Normalize to [-180, 180) first
    deg = std::fmod(deg, 360.0f);
    if (deg >= 180.0f)  deg -= 360.0f;
    if (deg < -180.0f)  deg += 360.0f;
    return static_cast<int8_t>(std::round((deg * 256.0f) / 360.0f));
}

static constexpr uint16_t BOT_PATH_MAGIC = 0xECFD;
struct BotNodeFlags
{
    static constexpr uint16_t NONE = 0;
    static constexpr uint16_t TURBO_PAD_HIGH = 1 << 0;
    static constexpr uint16_t SKIDMARKS_FRONT = 1 << 1;
    static constexpr uint16_t SKIDMARKS_BACK = 1 << 2;
    static constexpr uint16_t TERRAIN_MASK = 0x00F8;
    static constexpr uint16_t TURBO_PAD_LOW = 1 << 8;
    static constexpr uint16_t MASK_GRAB_STP = 1 << 9;
    static constexpr uint16_t JUMP = 1 << 10;
    static constexpr uint16_t DRIFT_LEFT = 1 << 11;
    static constexpr uint16_t DRIFT_RIGHT = 1 << 12;
    static constexpr uint16_t ENGINE_ECHO = 1 << 13;
    static constexpr uint16_t MID_AIR = 1 << 14;
    static constexpr uint16_t SINK_KART = 1 << 15;
};

struct BotPathSettings
{
    bool  useManualPath = false;
    bool  normalizeNodeDist = true;
    float nodeDistance = 4.0f;
    float sidewayOffset = 6.0f;
};


class BotNode
{
public:

    BotNode() = default;
    BotNode(const PSX::NavFrame& frame);
  
    std::vector<uint8_t> Serialize(const Vec3& nextPos) const;
    void RenderUI(int index, bool& deleteRequested);
    
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

    uint8_t  GetTerrain()      const { return m_terrain; }
    void     SetTerrain(uint8_t v) { m_terrain = v; }

    uint8_t  GetGoBackCount()      const { return m_goBackCount; }
    void     SetGoBackCount(uint8_t v) { m_goBackCount = v; }

    int  GetPathChange()      const { return m_pathChange; }
    void     SetPathChange(int v) { m_pathChange = v; }

    int  GetPathChangeIndex()      const { return m_pathChangeIndex; }
    void     SetPathChangeIndex(int v) { m_pathChangeIndex = v; }

    uint8_t  GetSpecialBits()      const { return m_specialBits; }
    void     SetSpecialBits(uint8_t v) { m_specialBits = v; }

private:
    Vec3 m_pos = {};
    float m_yaw = 0.0f; // rot[1]
    float m_pitch = 0.0f; // rot[0]
    float m_roll = 0.0f; // rot[2]

    uint16_t m_flags = 0;
    uint8_t m_terrain = TerrainType::ASPHALT;
    int m_pathChange = 0;
    int m_pathChangeIndex = 0;
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
    bool LoadFromOBJ(const std::filesystem::path& path, std::vector<Quadblock>& quadblocks);
    bool GeneratePath(std::vector<Vec3>& nodesPos, std::vector<Quadblock>& quadblocks);

    std::vector<uint8_t> Serialize() const;
    void RenderUI(int pathIndex);

    const std::vector<BotNode>& GetNodes() const { return m_nodes; }
    std::vector<BotNode>& GetNodes() { return m_nodes; }

    size_t GetNodeCount() const { return m_nodes.size(); }

    const BotNode& GetNode(size_t index) const { return m_nodes.at(index); }
    BotNode& GetNode(size_t index) { return m_nodes.at(index); }

    void AddNode(const BotNode& node) { m_nodes.push_back(node); }
    void InsertNode(size_t index, const BotNode& node) { m_nodes.insert(m_nodes.begin() + index, node); }
    void RemoveNode(size_t index) { m_nodes.erase(m_nodes.begin() + index); }

    uint16_t GetPhysUnk(size_t i) const { return m_physUnk[i]; }
    void     SetPhysUnk(size_t i, uint16_t v) { m_physUnk[i] = v; }

private:
    std::vector<BotNode> m_nodes;
    uint16_t m_physUnk[0x20] = {};
};

std::vector<Vec3> NormalizePos(const std::vector<Vec3>& pos, float dist);
std::vector<Vec3> GenerateLateralPath(const std::vector<BotNode>& nodes, float lateralOffset, std::vector<Quadblock>& quadblocks);
bool isAboveQuad(const Vec3& point, const Quadblock& quad, float& height);