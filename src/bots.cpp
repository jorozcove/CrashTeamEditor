#include "bots.h"

BotNode::BotNode(const PSX::NavFrame& frame)
{
    m_pos = ConvertPSXVec3(frame.pos, FP_ONE_GEO);
    m_pitch = BamToAngle(frame.rot[0]);
    m_yaw = BamToAngle(frame.rot[1]);
    m_roll = BamToAngle(frame.rot[2]);
    m_flags = frame.flags;
    m_terrain = (frame.flags & BotNodeFlags::TERRAIN_MASK) >> 3;
    m_pathChangeIndex = static_cast<int>(frame.pathChangeOpCode & 0x3FF) ;
    m_pathChange = static_cast<int>(frame.pathChangeOpCode >> 10);
    m_goBackCount = frame.goBackCount;
    m_specialBits = frame.specialBits;
}


std::vector<uint8_t> BotNode::Serialize(const Vec3& nextPos) const
{
    PSX::NavFrame frame = {};
    std::vector<uint8_t> buffer(sizeof(frame));
    frame.pos = ConvertVec3(m_pos, FP_ONE_GEO);
    frame.rot[0] = AngleToBam(m_pitch);
    frame.rot[1] = AngleToBam(m_yaw);
    frame.rot[2] = AngleToBam(m_roll);
    frame.rot[3] = -frame.rot[0]; // Not sure what this is
    frame.distXYZ = ConvertFloat((m_pos - nextPos).Length(), FP_ONE_GEO);
    frame.distXZ = ConvertFloat((m_pos - nextPos).LengthHorizontal(), FP_ONE_GEO);
    frame.flags = m_flags;
    frame.flags &= ~BotNodeFlags::TERRAIN_MASK; 
    frame.flags |= (m_terrain << 3) & BotNodeFlags::TERRAIN_MASK;
    frame.pathChangeOpCode = (static_cast<uint16_t>(m_pathChange) << 10) | (static_cast<uint16_t>(m_pathChangeIndex));
    frame.goBackCount = m_goBackCount;
    frame.specialBits = m_specialBits;
    std::memcpy(buffer.data(), &frame, sizeof(frame));
    return buffer;
}

BotPath::BotPath(const PSX::NavHeader& header, const std::vector<PSX::NavFrame>& frames)
{
    //m_offLastPoint = header.offLastPoint;
    std::copy(std::begin(header.physUnk), std::end(header.physUnk), std::begin(m_physUnk));

    m_nodes.reserve(frames.size());
    for (const auto& frame : frames)
    {
        m_nodes.emplace_back(frame);
    }   
}

void BotPath::Clear()
{
    m_nodes.clear();
}

bool BotPath::IsValid()
{
    return m_nodes.size() > 1;
}


bool BotPath::GeneratePath(std::vector<Vec3>& nodesPos, std::vector<Quadblock>& quadblocks)
{
    m_nodes.clear();
    if (nodesPos.empty()) { return false; }


    const size_t nodeCount = nodesPos.size();
    constexpr float GROUND_THRESHOLD = 2.0f;
    constexpr float NEARBY_THRESHOLD = 2.0f;
    constexpr float REVERB_THRESHOLD = 50.0f; // broader search for reverb, not just ground
    constexpr float SHARP_TURN_THRESHOLD = 15.0f; // degrees, for drift detection
    constexpr float DRIFT_BONUS_YAW = 45; // degrees, for drift yaw addition
    constexpr float SKIDMARK_LENGTH = 15.0f; // degrees, for drift yaw addition
    constexpr float BOT_SPEED = 25.0f;
    constexpr float SHARP_TURN_CIRCLE_SECONDS = 5.0f;
    constexpr float SHARP_TURN_DEG_PER_UNIT = 360.0f / (BOT_SPEED * SHARP_TURN_CIRCLE_SECONDS); // 2.4 deg/unit
    constexpr float DRIFT_MIN_DISTANCE = 25.0f;   // at least 2s worth of distance
    constexpr int   DRIFT_ANTICIPATION_NODES = 2; // start drift this many nodes before the sharp section

    // --- Helper: get the surface normal of a quadblock's sub-face under a given XZ position ---
// The quadblock has 4 sub-quads (q0..q3). We find which sub-quad contains the XZ point
// and return the normal of that face, computed from its vertices.
    auto GetSurfaceNormalAt = [&](const Quadblock& quad, const Vec3& pos) -> Vec3
        {
            const Vertex* const verts = quad.GetUnswizzledVertices();
            // Sub-quad vertex indices:
            // q0: p0,p1,p3,p4  q1: p1,p2,p4,p5
            // q2: p3,p4,p6,p7  q3: p4,p5,p7,p8
            const int subQuads[4][4] = {
                {0, 1, 3, 4},
                {1, 2, 4, 5},
                {3, 4, 6, 7},
                {4, 5, 7, 8}
            };

            // Find which sub-quad the XZ position falls inside, pick the closest if none match
            int bestQuad = 0;
            float bestDist = FLT_MAX;
            for (int q = 0; q < 4; q++)
            {
                const Vec3& a = verts[subQuads[q][0]].m_pos;
                const Vec3& c = verts[subQuads[q][3]].m_pos; // opposite corner
                float minX = std::min(a.x, c.x), maxX = std::max(a.x, c.x);
                float minZ = std::min(a.z, c.z), maxZ = std::max(a.z, c.z);
                if (pos.x >= minX && pos.x <= maxX && pos.z >= minZ && pos.z <= maxZ)
                {
                    bestQuad = q;
                    bestDist = 0.0f;
                    break;
                }
                // Fallback: distance from sub-quad center
                float cx = (a.x + c.x) * 0.5f;
                float cz = (a.z + c.z) * 0.5f;
                float dx = pos.x - cx, dz = pos.z - cz;
                float dist = dx * dx + dz * dz;
                if (dist < bestDist) { bestDist = dist; bestQuad = q; }
            }

            // Compute normal from the two triangles of the best sub-quad
            // Use p0,p1,p3 and p1,p3,p4 (top-left triangle and bottom-right triangle), average them
            const Vec3& v0 = verts[subQuads[bestQuad][0]].m_pos;
            const Vec3& v1 = verts[subQuads[bestQuad][1]].m_pos;
            const Vec3& v2 = verts[subQuads[bestQuad][2]].m_pos;
            const Vec3& v3 = verts[subQuads[bestQuad][3]].m_pos;

            auto Cross = [](const Vec3& a, const Vec3& b) -> Vec3 {
                return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
                };
            auto Sub = [](const Vec3& a, const Vec3& b) -> Vec3 {
                return { a.x - b.x, a.y - b.y, a.z - b.z };
                };
            auto Normalize = [](const Vec3& v) -> Vec3 {
                float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
                if (len < 1e-6f) return { 0.f, 1.f, 0.f };
                return { v.x / len, v.y / len, v.z / len };
                };

            Vec3 n0 = Cross(Sub(v2, v0), Sub(v3, v1)); // diagonal 0->2 cross diagonal 1->3
            Vec3 avg = Normalize(n0);

            if (avg.y < 0.0f) { avg.x = -avg.x; avg.y = -avg.y; avg.z = -avg.z; }
            //return avg;
            return quad.GetNormal();
        };

    // --- Helper: interpolate surface Y from quad vertices at XZ position ---
    auto GetSurfaceYAt = [&](const Quadblock& quad, const Vec3& pos) -> float
        {
            const Vertex* const verts = quad.GetUnswizzledVertices();
            const int subQuads[4][4] = {
                {0, 1, 3, 4}, {1, 2, 4, 5},
                {3, 4, 6, 7}, {4, 5, 7, 8}
            };

            // Find best sub-quad (same logic as above)
            int bestQuad = 0;
            float bestDist = FLT_MAX;
            for (int q = 0; q < 4; q++)
            {
                const Vec3& a = verts[subQuads[q][0]].m_pos;
                const Vec3& c = verts[subQuads[q][3]].m_pos;
                float minX = std::min(a.x, c.x), maxX = std::max(a.x, c.x);
                float minZ = std::min(a.z, c.z), maxZ = std::max(a.z, c.z);
                if (pos.x >= minX && pos.x <= maxX && pos.z >= minZ && pos.z <= maxZ)
                {
                    bestQuad = q; bestDist = 0.0f; break;
                }
                float cx = (a.x + c.x) * 0.5f, cz = (a.z + c.z) * 0.5f;
                float dx = pos.x - cx, dz = pos.z - cz;
                float dist = dx * dx + dz * dz;
                if (dist < bestDist) { bestDist = dist; bestQuad = q; }
            }

            // Bilinear interpolation of Y across the sub-quad
            const Vec3& tl = verts[subQuads[bestQuad][0]].m_pos; // top-left
            const Vec3& tr = verts[subQuads[bestQuad][1]].m_pos; // top-right
            const Vec3& bl = verts[subQuads[bestQuad][2]].m_pos; // bottom-left
            const Vec3& br = verts[subQuads[bestQuad][3]].m_pos; // bottom-right

            float rangeX = tr.x - tl.x;
            float rangeZ = bl.z - tl.z;
            float tx = (rangeX > 1e-6f) ? std::clamp((pos.x - tl.x) / rangeX, 0.f, 1.f) : 0.f;
            float tz = (rangeZ > 1e-6f) ? std::clamp((pos.z - tl.z) / rangeZ, 0.f, 1.f) : 0.f;

            float yTop = tl.y + tx * (tr.y - tl.y);
            float yBot = bl.y + tx * (br.y - bl.y);
            return yTop + tz * (yBot - yTop);
        };


    // --- Helper: find the best ground quadblock directly under a position ---
    auto FindGroundQuadUnder = [&](const Vec3& pos) -> const Quadblock*
        {
            const Quadblock* best = nullptr;
            float bestDist = FLT_MAX;
            for (const Quadblock& quad : quadblocks)
            {
                if (!(quad.GetFlags() & QuadFlags::GROUND)) { continue; }
                const BoundingBox& bb = quad.GetBoundingBox();
                // Broad XZ cull using bounding box first for performance
                if (pos.x < bb.min.x || pos.x > bb.max.x) { continue; }
                if (pos.z < bb.min.z || pos.z > bb.max.z) { continue; }
                // Get actual surface Y via vertex interpolation
                float surfaceY = GetSurfaceYAt(quad, pos);
                if (pos.y + GROUND_THRESHOLD < surfaceY) { continue; } // node is below the surface, skip
                float dist = pos.y - surfaceY;
                if (dist < bestDist) { bestDist = dist; best = &quad; }
            }
            return best;
        };

    // --- Helper: is the node grounded (within threshold of a ground quad) ---
    auto IsGrounded = [&](const Vec3& pos, const Quadblock* groundQuad) -> bool
        {
            if (!groundQuad) { return false; }
            float dist = pos.y - groundQuad->GetBoundingBox().max.y;
            return dist <= GROUND_THRESHOLD;
        };

    // --- Helper: any nearby quad matching a predicate ---
    auto HasNearbyQuad = [&](const Vec3& pos, float radius,
        const std::function<bool(const Quadblock&)>& predicate) -> bool
        {
            for (const Quadblock& quad : quadblocks)
            {
                const BoundingBox& bb = quad.GetBoundingBox();
                // Closest point on the AABB to pos — clamp each axis independently
                float cx = std::clamp(pos.x, bb.min.x, bb.max.x);
                float cy = std::clamp(pos.y, bb.min.y, bb.max.y);
                float cz = std::clamp(pos.z, bb.min.z, bb.max.z);
                float dx = pos.x - cx;
                float dy = pos.y - cy;
                float dz = pos.z - cz;
                float distSq = dx * dx + dy * dy + dz * dz;
                if (distSq <= radius * radius && predicate(quad)) { return true; }
            }
            return false;
        };

    //Helper : return an angle in between -180 and 180, modulo 360
    auto NormalizeAngle = [](float angle)
        {
            angle = std::fmod(angle + 180.0f, 360.0f);
            if (angle < 0)
                angle += 360.0f;
            return angle - 180.0f;
        };

    m_nodes.resize(nodeCount);

    // --- Pre-pass: detect air segments ---
    // A node is in the air if it is not grounded.
    // A jump starts at the last grounded node before becoming airborne.
    std::vector<const Quadblock*> groundQuads(nodeCount);
    std::vector<bool> grounded(nodeCount);
    for (size_t i = 0; i < nodeCount; i++)
    {
        groundQuads[i] = FindGroundQuadUnder(nodesPos[i]);
        grounded[i] = IsGrounded(nodesPos[i], groundQuads[i]);
    }

    /// --- Pre-pass: compute yaw and pitch ---
    std::vector<float> yaws(nodeCount);
    std::vector<float> pitches(nodeCount);
    for (size_t i = 0; i < nodeCount; i++)
    {
        const Vec3& curr = nodesPos[i];
        const Vec3& next = nodesPos[(i + 1) % nodeCount];
        float dx = next.x - curr.x;
        float dz = next.z - curr.z;
        float dy = next.y - curr.y;
        float horizDist = std::sqrt(dx * dx + dz * dz);

        // Yaw from delta position — unaffected by surface normal
        yaws[i] = std::atan2(dx, dz) * (180.0f / 3.14159265f);
        m_nodes[i].SetYaw(yaws[i]);

        // Pitch depends on whether the node is grounded or airborne
        const Quadblock* groundQuad = groundQuads[i];
        bool isGrounded = grounded[i];

        if (isGrounded && groundQuad)
        {
            // When grounded, the kart's local up = surface normal.
            // Pitch is the angle between the kart's forward axis and the surface plane.
            // Forward axis comes from yaw (which will later have drift bonus applied),
            // but at this stage we use the raw travel yaw — drift adjusts yaw[i] in
            // the drift pre-pass which runs after this, so pitch is recomputed after.
            Vec3 surfaceNormal = GetSurfaceNormalAt(*groundQuad, curr);

            // Build forward direction from current yaw in world space
            float yawRad = yaws[i] * (3.14159265f / 180.0f);
            Vec3 forward = { std::sin(yawRad), 0.0f, std::cos(yawRad) };

            // Project forward onto the surface plane (remove normal component)
            float fwdDotN = forward.x * surfaceNormal.x
                + forward.y * surfaceNormal.y
                + forward.z * surfaceNormal.z;
            Vec3 forwardOnSurface = {
                forward.x - fwdDotN * surfaceNormal.x,
                forward.y - fwdDotN * surfaceNormal.y,
                forward.z - fwdDotN * surfaceNormal.z
            };

            // Normalize the projected forward
            float len = std::sqrt(forwardOnSurface.x * forwardOnSurface.x
                + forwardOnSurface.y * forwardOnSurface.y
                + forwardOnSurface.z * forwardOnSurface.z);
            if (len > 1e-6f)
            {
                forwardOnSurface.x /= len;
                forwardOnSurface.y /= len;
                forwardOnSurface.z /= len;
            }

            // Pitch = angle the surface-projected forward makes with the horizontal plane
            // Positive = nose down (forward.y is negative when going downhill)
            pitches[i] = -std::asin(std::clamp(forwardOnSurface.y, -1.0f, 1.0f))
                * (180.0f / 3.14159265f);
        }
        else
        {
            // Airborne: local up = world up, pitch purely from travel direction
            pitches[i] = 0;
        }
    }


    // --- Pre-pass: drift ---

    // Issue 1 fix: convert yaw delta to angular velocity (degrees per unit of distance)
    // A full circle in 6 seconds at 25 units/s = 150 units of distance per full circle
    // = 360 / 150 = 2.4 degrees per unit -> anything tighter is a sharp turn


    // Compute distance between consecutive nodes
    std::vector<float> segmentDist(nodeCount);
    for (size_t i = 0; i < nodeCount; i++)
    {
        const Vec3& curr = nodesPos[i];
        const Vec3& next = nodesPos[(i + 1) % nodeCount];
        float dx = next.x - curr.x;
        float dy = next.y - curr.y;
        float dz = next.z - curr.z;
        segmentDist[i] = std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    // Compute angular velocity (deg/unit) at each node
    std::vector<float> angularVel(nodeCount);
    for (size_t i = 0; i < nodeCount; i++)
    {
        float dist = segmentDist[i];
        if (dist < 1e-6f) { angularVel[i] = 0.0f; continue; }
        float yawDelta = NormalizeAngle(yaws[(i + 1) % nodeCount] - yaws[i]);
        angularVel[i] = yawDelta / dist; // degrees per unit of distance
    }

    // Issue 2 fix: find raw sharp-turn sections first, then expand them
    // Step 1: mark nodes that are intrinsically sharp (by angular velocity)
    std::vector<int> driftDir(nodeCount, 0); // -1 = right, 0 = none, +1 = left
    for (size_t i = 0; i < nodeCount; i++)
    {
        if (!grounded[i]) { continue; }
        if (std::abs(angularVel[i]) >= SHARP_TURN_DEG_PER_UNIT)
            driftDir[i] = (angularVel[i] > 0.0f) ? 1 : -1;
    }

    // Step 2: find contiguous sharp sections, extend them to minimum distance,
    // and add anticipation nodes before each section
    // We iterate over nodes and collect "drift chunks"
    struct DriftChunk { size_t start; size_t end; int dir; }; // [start, end) inclusive
    std::vector<DriftChunk> chunks;

    size_t i = 0;
    while (i < nodeCount)
    {
        if (driftDir[i] == 0) { i++; continue; }

        // Found the beginning of a sharp section
        int dir = driftDir[i];
        size_t chunkStart = i;

        // Extend while still sharp and same direction
        while (i < nodeCount && driftDir[i] == dir) { i++; }
        size_t chunkEnd = i - 1; // last sharp node (inclusive)

        // Extend end forward until minimum distance is covered
        float coveredDist = 0.0f;
        for (size_t k = chunkStart; k <= chunkEnd; k++)
            coveredDist += segmentDist[k];

        size_t extendedEnd = chunkEnd;
        while (coveredDist < DRIFT_MIN_DISTANCE)
        {
            size_t next = (extendedEnd + 1) % nodeCount;
            if (!grounded[next]) { break; } // don't extend into air
            coveredDist += segmentDist[extendedEnd];
            extendedEnd = next;
            if (extendedEnd == chunkStart) { break; } // full loop guard
        }

        chunks.push_back({ chunkStart, extendedEnd, dir });
    }

    // Step 3: apply anticipation — shift each chunk's start back by DRIFT_ANTICIPATION_NODES
    // then write drift flags into m_nodes
    for (const DriftChunk& chunk : chunks)
    {
        // Walk back anticipation nodes, staying on ground
        size_t anticipatedStart = chunk.start;
        for (int k = 0; k < DRIFT_ANTICIPATION_NODES; k++)
        {
            size_t prev = (anticipatedStart + nodeCount - 1) % nodeCount;
            if (!grounded[prev]) { break; }
            anticipatedStart = prev;
        }

        // Apply drift flags to all nodes in [anticipatedStart .. chunk.end]
        size_t j = anticipatedStart;
        while (true)
        {
            uint16_t flags = m_nodes[j].GetFlags();
            if (chunk.dir < 0)
            {
                flags |= BotNodeFlags::DRIFT_LEFT;
                yaws[j] += DRIFT_BONUS_YAW;
            }
            else
            {
                flags |= BotNodeFlags::DRIFT_RIGHT;
                yaws[j] -= DRIFT_BONUS_YAW;
            }
            m_nodes[j].SetFlags(flags);
            if (j == chunk.end) { break; }
            j = (j + 1) % nodeCount;
        }
    }

    // After applying drift bonus yaw, recompute pitch for affected nodes
    // since pitch depends on the forward axis which changed with yaw
    for (size_t i = 0; i < nodeCount; i++)
    {
        uint16_t flags = m_nodes[i].GetFlags();
        bool isDrifting = (flags & (BotNodeFlags::DRIFT_LEFT | BotNodeFlags::DRIFT_RIGHT)) != 0;
        if (!isDrifting || !grounded[i] || !groundQuads[i]) { continue; }

        Vec3 surfaceNormal = GetSurfaceNormalAt(*groundQuads[i], nodesPos[i]);
        float yawRad = yaws[i] * (3.14159265f / 180.0f); // now includes drift bonus
        Vec3 forward = { std::sin(yawRad), 0.0f, std::cos(yawRad) };

        float fwdDotN = forward.x * surfaceNormal.x
            + forward.y * surfaceNormal.y
            + forward.z * surfaceNormal.z;
        Vec3 forwardOnSurface = {
            forward.x - fwdDotN * surfaceNormal.x,
            forward.y - fwdDotN * surfaceNormal.y,
            forward.z - fwdDotN * surfaceNormal.z
        };
        float len = std::sqrt(forwardOnSurface.x * forwardOnSurface.x
            + forwardOnSurface.y * forwardOnSurface.y
            + forwardOnSurface.z * forwardOnSurface.z);
        if (len > 1e-6f)
        {
            forwardOnSurface.x /= len;
            forwardOnSurface.y /= len;
            forwardOnSurface.z /= len;
        }
        pitches[i] = -std::asin(std::clamp(forwardOnSurface.y, -1.0f, 1.0f))
            * (180.0f / 3.14159265f);
    }


    uint8_t lastckpt = 0;
    for (size_t i = 0; i < nodeCount; i++)
    {
        BotNode& node = m_nodes[i];
        node.SetPos(nodesPos[i]);
        node.SetSpecialBits(0);
        node.SetPathChange(3); // no path change
        node.SetPathChangeIndex(static_cast<int>((i + 4) % nodeCount));
        const Quadblock* groundQuad = groundQuads[i];
        bool isGrounded = grounded[i];

        // --- Rotation ---
        
        node.SetPitch(pitches[i]);

        // Roll from surface normal: project the normal onto the node's right axis
        // Right axis is perpendicular to the forward (yaw) direction in XZ
        float yawRad = yaws[i] * (3.14159265f / 180.0f);
        // Forward direction from yaw (matching our convention: 0 = +Z, 90 = +X)
        Vec3 forward = { std::sin(yawRad), 0.0f, std::cos(yawRad) };
        Vec3 rightAxis = { std::cos(yawRad), 0.0f, -std::sin(yawRad) };

        if (groundQuad && isGrounded)
        {
            Vec3 normal = GetSurfaceNormalAt(*groundQuad, nodesPos[i]);

            // Remove the forward component from the normal to get the lateral tilt only
            float fwdDot = normal.x * forward.x + normal.y * forward.y + normal.z * forward.z;
            Vec3 normalLateral = {
                normal.x - fwdDot * forward.x,
                normal.y - fwdDot * forward.y,
                normal.z - fwdDot * forward.z
            };

            // The roll is the angle this lateral component makes with world up (0,1,0)
            // atan2 of the X component vs Y component gives signed roll
            float rollDeg = std::atan2(normalLateral.x, normalLateral.y) * (180.0f / 3.14159265f);
            //float rollDeg = std::atan2(normal.x, normal.y) * (180.0f / 3.14159265f);
            node.SetRoll(rollDeg);
        }
        else
        {
            node.SetRoll(0.0f);
        }

        // --- Terrain & go back count from ground quad ---
        
        if (groundQuad)
        {
            int cur_ckpt = groundQuad->GetCheckpoint();
            if (cur_ckpt >=  0) { lastckpt = static_cast<uint8_t>(std::clamp(cur_ckpt, 0, 255)); }
            node.SetGoBackCount(lastckpt);
            // Terrain from the quad directly underfoot
            node.SetTerrain(groundQuad->GetTerrain());

        }
        else
        {
            node.SetGoBackCount(lastckpt);
            node.SetTerrain(TerrainType::ASPHALT);
        }

        // --- Flags ---

        const Vec3& pos = nodesPos[i];
        uint16_t flags = node.GetFlags();

        // MID_AIR: not grounded
        if (!isGrounded)
        {
            flags |= BotNodeFlags::MID_AIR;
        }

        // JUMP: last grounded node before becoming airborne
        if (isGrounded)
        {
            bool nextAirborne = !grounded[(i + 1) % nodeCount];
            if (nextAirborne) { flags |= BotNodeFlags::JUMP; }
        }

        // SINK_KART: ground quad has water / fast water / mud terrain
        if (groundQuad)
        {
            uint8_t t = groundQuad->GetTerrain();
            if (t == TerrainType::WATER ||
                t == TerrainType::FAST_WATER ||
                t == TerrainType::MUD)
            {
                flags |= BotNodeFlags::SINK_KART;
            }
        }

        // TURBO_PAD_HIGH: nearby quad with TRIGGER_SCRIPT and Dirt terrain
        bool onTurboPad = HasNearbyQuad(pos, NEARBY_THRESHOLD, [](const Quadblock& q)
            {
                return (q.GetFlags() & QuadFlags::TRIGGER_SCRIPT) &&
                    (q.GetTerrain() == TerrainType::DIRT);
            });
        if (onTurboPad) { flags |= BotNodeFlags::TURBO_PAD_HIGH; }

        // TURBO_PAD_LOW: nearby quad with TRIGGER_SCRIPT and Grass terrain (super turbo pad)
        bool onSuperTurboPad = HasNearbyQuad(pos, NEARBY_THRESHOLD, [](const Quadblock& q)
            {
                return (q.GetFlags() & QuadFlags::TRIGGER_SCRIPT) &&
                    (q.GetTerrain() == TerrainType::GRASS);
            });
        if (onSuperTurboPad) { flags |= BotNodeFlags::TURBO_PAD_LOW; }

        // ENGINE_ECHO: any nearby quad (not necessarily ground) with REVERB flag
        bool hasReverb = HasNearbyQuad(pos, REVERB_THRESHOLD, [](const Quadblock& q)
            {
                return (q.GetFlags() & QuadFlags::REVERB) != 0;
            });
        if (hasReverb) { flags |= BotNodeFlags::ENGINE_ECHO; }

        // SKIDMARKS_BACK: when drifting
        bool drifting = (flags & (BotNodeFlags::DRIFT_LEFT | BotNodeFlags::DRIFT_RIGHT)) != 0;
        if (drifting) { flags |= BotNodeFlags::SKIDMARKS_BACK; }

        // SKIDMARKS_FRONT: drifting, or on a turbo/super turbo pad,
        // or the ~10 units after landing (transitioning from air to ground)
        bool prevAirborne = !grounded[(i + nodeCount - 1) % nodeCount];
        bool justLanded = isGrounded && prevAirborne;

        // Count how many nodes ago we landed to cover the ~10 unit window
        bool withinLandingWindow = false;
        if (isGrounded)
        {
            float distSinceLanding = 0.0f;
            for (size_t k = 1; k < nodeCount && distSinceLanding < SKIDMARK_LENGTH; k++)
            {
                size_t idx = (i + nodeCount - k) % nodeCount;
                if (!grounded[idx] || (m_nodes[idx].GetFlags() & (BotNodeFlags::TURBO_PAD_LOW | BotNodeFlags::TURBO_PAD_HIGH))) { withinLandingWindow = true; break; }
                size_t idxNext = (idx + 1) % nodeCount;
                Vec3 d = {
                    nodesPos[idxNext].x - nodesPos[idx].x,
                    nodesPos[idxNext].y - nodesPos[idx].y,
                    nodesPos[idxNext].z - nodesPos[idx].z
                };
                distSinceLanding += std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
            }
        }

        if (drifting || onTurboPad || onSuperTurboPad || withinLandingWindow)
        {
            flags |= BotNodeFlags::SKIDMARKS_FRONT;
        }

        node.SetFlags(flags);
    }
    return true;
}




std::vector<uint8_t> BotPath::Serialize() const
{
    PSX::NavHeader header = {};
    std::vector<uint8_t> buffer(sizeof(header));
    header.magic = BOT_PATH_MAGIC;
    header.numPoints = static_cast<uint16_t>(m_nodes.size()-1);
    header.unk1 = 0;
    header.posY = ConvertFloat(m_nodes[0].GetPosY(), FP_ONE_GEO);
    header.offLastPoint = 0;//m_offLastPoint;
    std::copy(std::begin(m_physUnk), std::end(m_physUnk), std::begin(header.physUnk)); // can't be removed (on crash cove, ramp fails), need to be understood
    std::memcpy(buffer.data(), &header, sizeof(header));

    for (int i = 0; i < m_nodes.size() - 1 ; i++)
    {
        int next_id = i == (m_nodes.size() - 2) ? 0 : i + 1; //2nd to last's next is the first. Last is handled differently
        const BotNode& node = m_nodes[i];
        const Vec3& nextPos = m_nodes[next_id].GetPos();
        auto nodeBytes = node.Serialize(nextPos); 
        buffer.insert(buffer.end(), nodeBytes.begin(), nodeBytes.end());
    }
    //Placeholder behavior for the last. Need to investigate how it works. It doesn't seem to be the distance to first.
    const BotNode& node = m_nodes[m_nodes.size() - 1];
    const Vec3& nextPos = m_nodes[0].GetPos();
    auto nodeBytes = node.Serialize(nextPos);
    buffer.insert(buffer.end(), nodeBytes.begin(), nodeBytes.end());
    return buffer;
}



