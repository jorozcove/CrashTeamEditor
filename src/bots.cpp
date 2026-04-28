#include "bots.h"

BotNode::BotNode(const PSX::NavFrame& frame)
{
    m_pos = ConvertPSXVec3(frame.pos, FP_ONE_GEO);
    m_pitch = BamToAngle(frame.rot[0]);
    m_yaw = BamToAngle(frame.rot[1]);
    m_roll = BamToAngle(frame.rot[2]);
    // rot[3] not stored

    m_unk2[0] = frame.unk2[0];
    m_unk2[1] = frame.unk2[1];
    m_flags = frame.flags;
    m_pathChangeOpCode = frame.pathChangeOpCode;
    m_goBackCount = frame.goBackCount;
    m_specialBits = frame.specialBits;
}


std::vector<uint8_t> BotNode::Serialize() const
{
    PSX::NavFrame frame = {};
    std::vector<uint8_t> buffer(sizeof(frame));
    frame.pos = ConvertVec3(m_pos, FP_ONE_GEO);
    frame.rot[0] = AngleToBam(m_pitch);
    frame.rot[1] = AngleToBam(m_yaw);
    frame.rot[2] = AngleToBam(m_roll);
    frame.rot[3] = -frame.rot[0]; // Not sure what this is
    frame.unk2[0] = m_unk2[0];
    frame.unk2[1] = m_unk2[1];
    frame.flags = m_flags;
    frame.pathChangeOpCode = m_pathChangeOpCode;
    frame.goBackCount = m_goBackCount;
    frame.specialBits = m_specialBits;
    std::memcpy(buffer.data(), &frame, sizeof(frame));
    return buffer;
}

BotPath::BotPath(const PSX::NavHeader& header, const std::vector<PSX::NavFrame>& frames)
{
    m_magic = header.magic;
    m_posY = header.posY;
    m_offLastPoint = header.offLastPoint;
    std::copy(std::begin(header.physUnk), std::end(header.physUnk), std::begin(m_physUnk));

    m_nodes.reserve(frames.size());
    for (const auto& frame : frames)
        m_nodes.emplace_back(frame);
}

void BotPath::Clear()
{
    m_nodes.clear();
}

bool BotPath::IsValid()
{
    return m_nodes.size() > 0;
}

std::vector<uint8_t> BotPath::Serialize() const
{
    PSX::NavHeader header = {};
    std::vector<uint8_t> buffer(sizeof(header));
    header.magic = m_magic;
    header.numPoints = static_cast<uint16_t>(m_nodes.size()-1);
    header.posY = m_posY;
    header.offLastPoint = m_offLastPoint;
    std::copy(std::begin(m_physUnk), std::end(m_physUnk), std::begin(header.physUnk));
    std::memcpy(buffer.data(), &header, sizeof(header));

    for (const BotNode& node : m_nodes)
    {
        auto nodeBytes = node.Serialize();
        buffer.insert(buffer.end(), nodeBytes.begin(), nodeBytes.end());
    }
    return buffer;
}