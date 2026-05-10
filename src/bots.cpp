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
    frame.rot[0] = AngleToBam(45.0f);
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

bool isAboveQuad(const Vec3& point, const Quadblock& quad, float& height)
    //Helper function : is a point above a quadblock ?
    //Modify height
{
    const Vertex* verts = quad.GetUnswizzledVertices();
    for (const std::array<size_t, 3>&ids : quad.GetCollTriFacesIndexes())
    {
        const Vec3& A = verts[ids[0]].m_pos;
        const Vec3& B = verts[ids[1]].m_pos;
        const Vec3& C = verts[ids[2]].m_pos;
        const float d1 = (B.x - A.x) * (point.z - A.z) - (B.z - A.z) * (point.x - A.x);
        const float d2 = (C.x - B.x) * (point.z - B.z) - (C.z - B.z) * (point.x - B.x);
        const float d3 = (A.x - C.x) * (point.z - C.z) - (A.z - C.z) * (point.x - C.x);
        if ((d1 < -EPSILON || d2 < -EPSILON || d3 < -EPSILON) && (d1 > EPSILON || d2 > EPSILON || d3 > EPSILON))
        {
            continue; // Point outisde of Tri
        }

        const float denom = (B.z - C.z) * (A.x - C.x) + (C.x - B.x) * (A.z - C.z);
        if (std::abs(denom) < EPSILON)
            continue; // degenerate triangle, skip

        const float u = ((B.z - C.z) * (point.x - C.x) + (C.x - B.x) * (point.z - C.z)) / denom;
        const float v = ((C.z - A.z) * (point.x - C.x) + (A.x - C.x) * (point.z - C.z)) / denom;
        const float w = 1.0f - u - v;

        height = u * A.y + v * B.y + w * C.y;
        return true;
    }
    return false;
}


bool BotPath::GeneratePath(std::vector<Vec3>& nodesPos, std::vector<Quadblock>& quadblocks)
{
    m_nodes.clear();
    if (nodesPos.empty()) { return false; }


    const size_t nodeCount = nodesPos.size();
    constexpr float GROUND_THRESHOLD = 8.0f;
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

    // Pass 1 : Detect AirTime + Snap to Ground.
    std::vector<const Quadblock*> groundQuads(nodeCount);
    std::vector<bool> grounded(nodeCount);
    for (size_t i = 0; i < nodeCount; i++)
    {
        Vec3        pos = nodesPos[i];
        float       bestDist = GROUND_THRESHOLD;
        float       bestSurfaceY = pos.y;

        for (const Quadblock& quad : quadblocks)
        {
            if (!(quad.GetFlags() & QuadFlags::GROUND))
                continue;

            const BoundingBox& bb = quad.GetBoundingBox();
            if (pos.x < bb.min.x || pos.x > bb.max.x) continue;
            if (pos.z < bb.min.z || pos.z > bb.max.z) continue;

            float height = 0.0f;
            bool above = isAboveQuad(pos, quad, height);
            if (!above)
                continue;

            const float dist = std::abs(pos.y - height);
            if (dist > bestDist)
                continue;
            bestDist = dist;
            groundQuads[i] = &quad;
            pos.y = height;
        }
        grounded[i] = bestDist < GROUND_THRESHOLD;
        m_nodes[i].SetPos(pos); 
    }

    /// --- Pre-pass: compute yaw ---
    std::vector<float> yaws(nodeCount);
    std::vector<float> pitches(nodeCount);
    for (size_t i = 0; i < nodeCount; i++)
    {
        const Vec3& curr = nodesPos[i];
        const Vec3& next = nodesPos[(i + 1) % nodeCount];
        float dx = next.x - curr.x;
        float dz = next.z - curr.z;
        float dy = next.y - curr.y;

        // Yaw from delta position — unaffected by surface normal
        yaws[i] = std::atan2(dx, dz) * (180.0f / 3.14159265f);
        m_nodes[i].SetYaw(yaws[i]);
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
        segmentDist[i] = (next-curr).Length();
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

        Vec3 surfaceNormal = groundQuads[i]->GetNormal();
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
            Vec3 normal = groundQuads[i]->GetNormal();

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
    // Crash if called with invalid nodes. Never serialize empty path.
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


std::vector<Vec3> NormalizePos(const std::vector<Vec3>& pos, float dist) {
    int numPoint = pos.size();
    if (numPoint < 2 || dist <= 0.0f) return pos;

    auto catmullRomAlpha = [](const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, float t, float alpha = 0.5f) -> Vec3 {
        auto getT = [alpha](float t, const Vec3& p0, const Vec3& p1) -> float {
            float d = (p1 - p0).Length(); // Assuming Vec3 has a Length() method
            return t + std::pow(std::max(d, 1e-6f), alpha);
            };

        const float t0 = 0.0f;
        const float t1 = getT(t0, p0, p1);
        const float t2 = getT(t1, p1, p2);
        const float t3 = getT(t2, p2, p3);

        const float s = t1 + t * (t2 - t1);

        const Vec3 A1 = p0 * ((t1 - s) / (t1 - t0)) + p1 * ((s - t0) / (t1 - t0));
        const Vec3 A2 = p1 * ((t2 - s) / (t2 - t1)) + p2 * ((s - t1) / (t2 - t1));
        const Vec3 A3 = p2 * ((t3 - s) / (t3 - t2)) + p3 * ((s - t2) / (t3 - t2));

        const Vec3 B1 = A1 * ((t2 - s) / (t2 - t0)) + A2 * ((s - t0) / (t2 - t0));
        const Vec3 B2 = A2 * ((t3 - s) / (t3 - t1)) + A3 * ((s - t1) / (t3 - t1));

        return B1 * ((t2 - s) / (t2 - t1)) + B2 * ((s - t1) / (t2 - t1));
        };

    auto getPoint = [&](int i) -> const Vec3& {
        return pos[((i % numPoint) + numPoint) % numPoint];
        };

    // 1. Generate Dense Samples
    const int stepsPerSegment = 64;
    std::vector<Vec3> denseSamples;
    denseSamples.reserve(numPoint * stepsPerSegment);

    for (int i = 0; i < numPoint; i++) {
        const Vec3& p0 = getPoint(i - 1);
        const Vec3& p1 = getPoint(i);
        const Vec3& p2 = getPoint(i + 1);
        const Vec3& p3 = getPoint(i + 2);

        for (int step = 0; step < stepsPerSegment; step++) {
            float t = (float)step / (float)stepsPerSegment;
            denseSamples.push_back(catmullRomAlpha(p0, p1, p2, p3, t));
        }
    }
    // Add the very first point again at the end to "close" the dense loop for the distance walker
    denseSamples.push_back(denseSamples.front());

    // 2. Distribute points by 'dist'
    std::vector<Vec3> result;
    float accumulated = 0.0f;

    // We start by adding the first point
    result.push_back(denseSamples.front());

    for (size_t i = 1; i < denseSamples.size(); i++) {
        Vec3 segment = denseSamples[i] - denseSamples[i - 1];
        float segLen = segment.Length();
        if (segLen <= 0.00001f) continue;

        accumulated += segLen;

        while (accumulated >= dist) {
            float overshot = accumulated - dist;
            float ratio = (segLen - overshot) / segLen;
            Vec3 newPoint = denseSamples[i - 1] + segment * ratio;

            result.push_back(newPoint);

            // Prepare for next potential point in same segment
            accumulated = overshot;
            // In a loop, we usually don't want the last point to overlap the first.
            // If the last point is extremely close to the first, you might want to break.
        }
    }

    // Since it's a loop, the very last point in 'result' might be very close to result[0].
    // Depending on your needs, you might want to pop_back() the last point if it's too close.
    if (result.size() > 1) {
        if ((result.back() - result.front()).Length() < dist * 0.5f) {
            result.pop_back();
        }
    }

    return result;
}

std::vector<Vec3> GenerateLateralPath(const std::vector<BotNode>& nodes, float lateralOffset, std::vector<Quadblock>& quadblocks)
{
    if (nodes.size() < 2)
    {
        std::vector<Vec3> fallback;
        fallback.reserve(nodes.size());
        for (const BotNode& node : nodes)
            fallback.push_back(node.GetPos());
        return fallback;
    }

    float sign = lateralOffset < 0 ? -1.0f : 1.0f;
    const Vec3 up(0.0f, 1.0f, 0.0f);

    // Check if a given XZ position falls within the XZ bounds of any quadblock with the given checkpoint ID
    auto isAboveAnyQuadblock = [&](const Vec3& testPos, int checkpointID, float& height) -> bool
        {
            for (const Quadblock& quad : quadblocks)
            {
                if (quad.GetCheckpoint() > checkpointID + 1 || quad.GetCheckpoint() < checkpointID - 1)
                    continue;
                if (isAboveQuad(testPos, quad, height))
                    return true;
            }
            return false;
        };

    std::vector<Vec3> result;
    result.reserve(nodes.size());

    float currLateralOffset = lateralOffset;
    constexpr float reductionFactor = 0.8f;
    constexpr int   maxAttempts = 10;

    for (int i = 0; i < nodes.size(); i++)
    {
        const Vec3 nodePos = nodes[i].GetPos();
        const int  checkpointID = static_cast<int>(nodes[i].GetGoBackCount());

        Vec3 forward = nodes[(i == nodes.size() - 1) ? 0 : i + 1].GetPos() - nodes[i].GetPos();
        forward.Normalize();

        Vec3 right = forward.Cross(up);
        right.Normalize();
        if (right.Length() < EPSILON)
            right = Vec3(1.0f, 0.0f, 0.0f);
        float _ = 0.0f;
        if (!isAboveAnyQuadblock(nodePos, checkpointID, _))
        {
            result.push_back(nodePos + right * currLateralOffset);
            continue;
        }

        float tempLateralOffset = sign * std::fmin(std::abs(currLateralOffset) / reductionFactor, std::abs(lateralOffset));
        Vec3 candidatePos;
        float target_height = 0.0f;

        for (int attempt = 0; attempt < maxAttempts; attempt++)
        {
            candidatePos = nodePos + right * tempLateralOffset;
            if (isAboveAnyQuadblock(candidatePos, checkpointID, target_height))
            {
                currLateralOffset = tempLateralOffset;
                //candidatePos.y = target_height;
                break;
            }   
            tempLateralOffset *= reductionFactor;
        }
        result.push_back(candidatePos);
    }
    return result;
}
