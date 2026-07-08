#include "level.h"
#include "ui.h"
#include "psx_types.h"
#include "io.h"
#include "utils.h"
#include "geo.h"
#include "process.h"
#include "gui_render_settings.h"
#include "renderer.h"
#include "vistree.h"
#include "text3d.h"
#include "levdataextractor.h"


#include <filesystem>
#include <iostream>
#include <fstream>
#include <unordered_set>
#include <array>
#include <system_error>
#include <set>
#include <map>
#include <algorithm>
#include <cstring>
#include <stb_image_write.h>

bool Level::Load(const std::filesystem::path& filename, bool isLevel)
{
	Clear(true);
	if (!filename.has_filename() || !filename.has_extension()) { return false; }
	std::filesystem::path ext = filename.extension();
	if (ext == ".lev") { return LoadLEV(filename); }
	if (ext == ".obj") { return LoadOBJ(filename, isLevel); }
	return false;
}

bool Level::Save(const std::filesystem::path& path)
{
	return false;// SaveLEV(path);
}

bool Level::IsLoaded() const
{
	return m_loaded;
}

void Level::OpenHotReloadWindow()
{
	m_showHotReloadWindow = true;
}

void Level::OpenModelExtractorWindow()
{
	m_showModelExtractorWindow = true;
}


void Level::Clear(bool clearErrors)
{
	m_loaded = false;
	m_showHotReloadWindow = false;
	m_showModelExtractorWindow = false;
	m_showExtractorLogWindow = false;
	for (size_t i = 0; i < NUM_DRIVERS; i++) { m_spawn[i] = Spawn(); }
	for (size_t i = 0; i < NUM_GRADIENT; i++) { m_skyGradient[i] = ColorGradient(); }
	if (clearErrors)
	{
		m_showLogWindow = false;
		m_logMessage.clear();
		m_invalidQuadblocks.clear();
	}
	m_configFlags = LevConfigFlags::NONE;
	m_clearColor = Color();
	m_stars = {};
	m_stars.zDepth = static_cast<uint16_t>(OT_SIZE) - 2;
	m_name.clear();
	m_hotReloadLevPath.clear();
	m_hotReloadVRMPath.clear();
	m_quadblocks.clear();
	m_checkpoints.clear();
	m_bsp.Clear();
	m_materialToQuadblocks.clear();
	m_materialToTexture.clear();
	m_checkpointPaths.clear();
	m_tropyGhost.clear();
	m_oxideGhost.clear();
	m_animTextures.clear();
	m_rendererQueryPoint = Vec3();
	m_rendererSelectedQuadblockIndexes.clear();
	//m_genVisTree = false;
	m_bspVis.Clear();
	m_maxQuadPerLeaf = 31;
	m_maxLeafAxisLength = 64.0f;
	m_visTreeSettings = VisTreeSettings();
	m_pythonConsole.clear();
	m_saveScript = false;
	m_vrm.clear();
	m_lastAnimTextureCount = 0;
	DeleteMaterials(this);
	m_skybox.Clear();
	m_splitLines[0] = 0.0;
	m_splitLines[1] = 0.0;
	m_jumpYSpeedCap = 0;
	m_instances.clear();
	m_importedModels.clear();
	m_parsedModelCache.clear();
	m_levData.clear();
	m_vramData.clear();
	std::error_code ec;
	std::filesystem::remove_all(std::filesystem::temp_directory_path() / "CTE_tex_cache", ec);

	m_openInstanceIndex = -1;
	m_closeInstanceIndex = -1;
	for (int i = 0; i < 3; i++)
	{
		m_botPaths[i].Clear();
	}

	for (Model* model : m_models)
	{
		if (model) { model->Clear(model != m_models[LevelModels::LEVEL]); }
	}
}

bool Level::ImportModel(const std::filesystem::path& ctrmodelPath)
{
	// Read entire file into memory
	std::ifstream file(ctrmodelPath, std::ios::binary);
	if (!file) { return false; }
	std::vector<uint8_t> ctrmodelData{
		std::istreambuf_iterator<char>(file),
		std::istreambuf_iterator<char>()
	};
	file.close();

	// Parse model name from header
	const SH::CtrModel* header = reinterpret_cast<const SH::CtrModel*>(ctrmodelData.data());
	const PSX::Model* model = reinterpret_cast<const PSX::Model*>(ctrmodelData.data() + header->modelOffset);
	std::string name(model->name, strnlen(model->name, sizeof(model->name)));

	printf("Imported model: %s (%zu bytes)\n", name.c_str(), ctrmodelData.size());
	m_importedModels[name] = std::move(ctrmodelData);

	return true;
}

const std::string& Level::GetName() const
{
	return m_name;
}

std::vector<Quadblock>& Level::GetQuadblocks()
{
	return m_quadblocks;
}

BSP& Level::GetBSP()
{
	return m_bsp;
}

BitMatrix& Level::GetVisTree()
{
	return m_bspVis;
}

std::vector<Checkpoint>& Level::GetCheckpoints()
{
	return m_checkpoints;
}

std::vector<Path>& Level::GetCheckpointPaths()
{
	return m_checkpointPaths;
}

std::vector<BotNode>& Level::GetBotPath(int i)
{
	return m_botPaths[i].GetNodes();
}

const std::filesystem::path& Level::GetParentPath() const
{
	return m_parentPath;
}

std::vector<std::string> Level::GetMaterialNames() const
{
	std::vector<std::string> names;
	names.reserve(m_materialToQuadblocks.size());
	for (const auto& [key, value] : m_materialToQuadblocks) { names.push_back(key); }
	return names;
}

std::vector<size_t> Level::GetMaterialQuadblockIndexes(const std::string& material) const
{
	if (!m_materialToQuadblocks.contains(material)) { return std::vector<size_t>(); }
	return m_materialToQuadblocks.at(material);
}

std::tuple<std::vector<Quadblock*>, Vec3> Level::GetRendererSelectedData()
{
	std::vector<Quadblock*> quadblocks;
	quadblocks.reserve(m_rendererSelectedQuadblockIndexes.size());
	for (size_t index : m_rendererSelectedQuadblockIndexes)
	{
		if (index < m_quadblocks.size()) { quadblocks.push_back(&m_quadblocks[index]); }
	}
	return std::make_tuple(std::move(quadblocks), m_rendererQueryPoint);
}

Model* Level::GetLevelModel()
{
	return m_models[LevelModels::LEVEL];
}

Model* Level::GetBspModel()
{
	return m_models[LevelModels::BSP];
}

Model* Level::GetSpawnModel()
{
	return m_models[LevelModels::SPAWN];
}

Model* Level::GetCheckpointModel()
{
	return m_models[LevelModels::CHECKPOINT];
}

Model* Level::GetBotModel()
{
	return m_models[LevelModels::BOT];
}

Model* Level::GetSelectedModel()
{
	return m_models[LevelModels::SELECTED];
}

Model* Level::GetMultiSelectedModel()
{
	return m_models[LevelModels::MULTI_SELECTED];
}

Model* Level::GetFilterModel()
{
	return m_models[LevelModels::FILTER];
}

Model* Level::GetInstancesModel()
{
	return m_models[LevelModels::INSTANCES];
}


bool Level::GenerateSpawn(float colSpacing, float rowSpacing)
{
	if (m_checkpoints.size() < 2)
		return false;

	Vec3 up = { 0.0f, 1.0f, 0.0f };
	Vec3 cp0 = m_checkpoints[0].GetPos();
	Vec3 cp1 = m_checkpoints[1].GetPos();
	Vec3 center = m_checkpoints[m_checkpoints[0].GetDown()].GetPos();
	Vec3 forward = cp1 - cp0;
	forward.y = 0;
	float yaw = - std::atan2(forward.z, forward.x) * (180.0f / 3.14159265f);
	yaw = std::fmod(yaw, 360.0f);
	forward.Normalize();
	Vec3 right = forward.Cross(up);

	for (int row = 0; row < 2; row++)
	{
		for (int col = 0; col < 4; col++)
		{
			int index = row * 4 + col;
			float lateralOffset = (col - 1.5f) * colSpacing;
			float forwardOffset = (row - 0.5f) * rowSpacing;
			Vec3 pos = center + right * lateralOffset + forward * forwardOffset;
			bool isInRange = false;
			int lastCkpt = m_checkpoints[0].GetDown();
			int prevCkpt = m_checkpoints[lastCkpt].GetDown();
			for (const Quadblock& quad : m_quadblocks)
			{
				if (quad.GetCheckpoint() != lastCkpt && quad.GetCheckpoint() != prevCkpt)
					continue;
				if (isAboveQuad(pos, quad, pos.y))
					isInRange = true;
			}
			if (!isInRange)
				return false;
			m_spawn[index].pos = pos;
			m_spawn[index].rot.x = 0.0f;
			m_spawn[index].rot.y = yaw;
			m_spawn[index].rot.z = 0.0f;
		}
	}
	return true;
}

std::string Level::GenerateUniqueInstanceName(const std::string& name) const
{
	std::string stripped = name;
	size_t hashPos = stripped.rfind('#');
	if (hashPos != std::string::npos && hashPos + 1 < stripped.size())
	{
		std::string suffix = stripped.substr(hashPos + 1);
		if (!suffix.empty() && std::all_of(suffix.begin(), suffix.end(), ::isdigit))
			stripped = stripped.substr(0, hashPos);
	}

	int maxN = 0;
	for (const auto& inst : m_instances)
	{
		const std::string& n = inst.GetName();
		if (n.size() > stripped.size() + 1 &&
			n.compare(0, stripped.size(), stripped) == 0 &&
			n[stripped.size()] == '#')
		{
			std::string suffix = n.substr(stripped.size() + 1);
			if (!suffix.empty() && std::all_of(suffix.begin(), suffix.end(), ::isdigit))
			{
				int val = std::stoi(suffix);
				if (val > maxN) maxN = val;
			}
		}
	}

	return stripped + "#" + std::to_string(maxN + 1);
}

bool Level::GenerateInstanceRow(int checkpointIndex, size_t instanceIndex, int numInstances, float spacing, bool deleteAfter)
{
	if (m_checkpoints.empty())
		return false;
	if (instanceIndex >= m_instances.size())
		return false;
	if (checkpointIndex < 0 || checkpointIndex >= static_cast<int>(m_checkpoints.size()))
		return false;
	if (numInstances < 1)
		return false;

	const Checkpoint& cp = m_checkpoints[checkpointIndex];
	Vec3 center = cp.GetPos();

	int down = cp.GetDown();
	int up = cp.GetUp();
	Vec3 forward;
	if (down != NONE_CHECKPOINT_INDEX && down >= 0 && down < static_cast<int>(m_checkpoints.size()))
		forward = m_checkpoints[down].GetPos() - center;
	else if (up != NONE_CHECKPOINT_INDEX && up >= 0 && up < static_cast<int>(m_checkpoints.size()))
		forward = center - m_checkpoints[up].GetPos();
	else
		forward = Vec3(0.0f, 0.0f, 1.0f);

	Vec3 upVec = { 0.0f, 1.0f, 0.0f };
	forward.y = 0;
	float yaw = -std::atan2(forward.z, forward.x) * (180.0f / 3.14159265f);
	yaw = std::fmod(yaw, 360.0f);
	forward.Normalize();
	Vec3 right = forward.Cross(upVec);

	Instance original = m_instances[instanceIndex];
	size_t insertPos = instanceIndex + 1;

	for (int col = 0; col < numInstances; col++)
	{
		float lateralOffset = (col - (numInstances - 1) * 0.5f) * spacing;
		Vec3 pos = center + right * lateralOffset;

		Instance newInstance = original;

		newInstance.SetName(GenerateUniqueInstanceName(original.GetName()));

		newInstance.SetPos(pos);
		newInstance.SetRot(Vec3(0.0f, yaw, 0.0f));

		m_instances.insert(m_instances.begin() + insertPos + col, newInstance);
	}

	if (deleteAfter)
	{
		m_instances.erase(m_instances.begin() + instanceIndex);
	}

	return true;
}

bool Level::GenerateBSP()
{
	std::vector<size_t> quadIndexes;
	for (size_t i = 0; i < m_quadblocks.size(); i++) { quadIndexes.push_back(i); }
	m_bsp.Clear();
	m_bsp.SetQuadblockIndexes(quadIndexes);
	m_bsp.Generate(m_quadblocks, m_maxQuadPerLeaf, m_maxLeafAxisLength);
	if (m_bsp.IsValid())
	{
		GenerateRenderBspData();
		return true;
	}
	m_bsp.Clear();
	return false;
}


bool Level::GenerateVisTreeOnly(bool simpleVisTree, float distanceNearClip, float distanceFarClip)
{
	if (m_bsp.IsValid())
	{
		VisTreeSettings settings;
		settings.farClipDistance = distanceFarClip;
		settings.nearClipDistance = distanceNearClip;
		settings.centerOnlySamples = simpleVisTree; 
		settings.commutativeRays = false;
		m_bspVis = GenerateVisTree(m_quadblocks, &m_bsp, settings);
		return true;
	}
	return false;
}


bool Level::GenerateVisTreeOnly()
{
	if (m_bsp.IsValid())
	{
		m_bspVis = GenerateVisTree(m_quadblocks, &m_bsp, m_visTreeSettings);
		return true;
	}
	return false;
}


std::vector<Vec3> Level::LoadPath(const std::filesystem::path& path)
{
	std::ifstream file(path);
	if (!file.is_open())
		return {};

	std::vector<Vec3>                        rawVertices;
	std::unordered_map<int, int>             adjacency;   // edge map: from -> to (1-based)
	bool                                     inFirstObject = false;

	std::string line;
	while (std::getline(file, line))
	{
		if (line.empty() || line[0] == '#')
			continue;

		std::istringstream ss(line);
		std::string        token;
		ss >> token;

		if (token == "o")
		{
			// Only parse the first object
			if (!inFirstObject)
				inFirstObject = true;
			else
				break;
		}
		else if (token == "v" && inFirstObject)
		{
			float x, y, z;
			ss >> x >> y >> z;
			rawVertices.emplace_back(x, y, z);
		}
		else if (token == "l" && inFirstObject)
		{
			int a, b;
			if (ss >> a >> b)
				adjacency[a] = b;  // directed edge a -> b (OBJ indices are 1-based)
		}
	}

	if (rawVertices.empty() || adjacency.empty())
		return rawVertices;

	// Find the start of the chain: a vertex that appears as a source but never as a destination
	std::unordered_set<int> destinations;
	for (auto& [from, to] : adjacency)
		destinations.insert(to);

	int start = -1;
	for (auto& [from, to] : adjacency)
	{
		if (destinations.find(from) == destinations.end())
		{
			start = from;
			break;
		}
	}

	// Fallback: if it's a closed loop, just pick any start
	if (start == -1 && !adjacency.empty())
		start = adjacency.begin()->first;

	// Walk the chain in edge order
	std::vector<Vec3> ordered;
	ordered.reserve(rawVertices.size());

	int current = start;
	while (adjacency.count(current))
	{
		// OBJ indices are 1-based
		ordered.push_back(rawVertices[current - 1]);
		int next = adjacency[current];
		adjacency.erase(current);  // prevent infinite loops on malformed data
		current = next;
	}
	// Push the final vertex (the chain end that has no outgoing edge)
	if (current >= 1 && current <= static_cast<int>(rawVertices.size()))
		ordered.push_back(rawVertices[current - 1]);

	return ordered;
}


void Level::GenerateBotPathChangeCode()
{
	// For each node on a path, find the closest node on the target path
	// and set the PathChange and PathChangeIndex accordingly.
	auto findClosestNode = [](const std::vector<BotNode>& targetNodes, const Vec3& pos) -> int
		{
			int   bestIndex = 0;
			float bestDist = FLT_MAX;
			for (int i = 0; i < static_cast<int>(targetNodes.size()); i++)
			{
				const Vec3& targetPos = targetNodes[i].GetPos();
				const float dx = pos.x - targetPos.x;
				const float dy = pos.y - targetPos.y;
				const float dz = pos.z - targetPos.z;
				const float dist = dx * dx + dy * dy + dz * dz; // squared, no need for sqrt
				if (dist < bestDist)
				{
					bestDist = dist;
					bestIndex = i;
				}
			}
			return bestIndex;
		};

	// Validate that all 3 paths are valid before proceeding
	for (int i = 0; i < 3; i++)
	{
		if (!m_botPaths[i].IsValid())
		{
			// Can't generate path change codes without all 3 paths
			return;
		}
	}

	const std::vector<BotNode>& leftNodes = m_botPaths[0].GetNodes();
	const std::vector<BotNode>& middleNodes = m_botPaths[1].GetNodes();
	const std::vector<BotNode>& rightNodes = m_botPaths[2].GetNodes();

	// --- Path 0 (Left): can only switch to Middle (1) ---
	for (int i = 0; i < static_cast<int>(leftNodes.size()); i++)
	{
		BotNode& node = m_botPaths[0].GetNode(i);
		const int closestMid = findClosestNode(middleNodes, node.GetPos());
		node.SetPathChange(1);
		node.SetPathChangeIndex(closestMid);
	}

	// --- Path 2 (Right): can only switch to Middle (1) ---
	for (int i = 0; i < static_cast<int>(rightNodes.size()); i++)
	{
		BotNode& node = m_botPaths[2].GetNode(i);
		const int closestMid = findClosestNode(middleNodes, node.GetPos());
		node.SetPathChange(1);
		node.SetPathChangeIndex(closestMid);
	}

	// --- Path 1 (Middle): can switch to Left (0) or Right (2) ---
	// Alternate between left and right to distribute switches evenly,
	// so the AI doesn't always prefer one side.
	for (int i = 0; i < static_cast<int>(middleNodes.size()); i++)
	{
		BotNode& node = m_botPaths[1].GetNode(i);
		if (i % 2 == 0)
		{
			// Switch to Left
			const int closestLeft = findClosestNode(leftNodes, node.GetPos());
			node.SetPathChange(0);
			node.SetPathChangeIndex(closestLeft);
		}
		else
		{
			// Switch to Right
			const int closestRight = findClosestNode(rightNodes, node.GetPos());
			node.SetPathChange(2);
			node.SetPathChangeIndex(closestRight);
		}
	}
}


void Level::GenerateBotPathLeft()
{
	std::vector<Vec3> pos;
	for (BotNode& node : m_botPaths[0].GetNodes())
	{
		pos.push_back(node.GetPos());
	}
	m_botPaths[0].GeneratePath(pos, m_quadblocks);
}

bool Level::GenerateCheckpoints()
{
	if (m_checkpointPaths.empty()) { return false; }

	for (const Path& path : m_checkpointPaths) { if (!path.IsReady()) { return false; } }

	ResetFilter();
	for (size_t i = 0; i < m_quadblocks.size(); i++)
	{
		m_quadblocks[i].SetCheckpoint(-1);
	}
	size_t checkpointIndex = 0;
	std::vector<size_t> linkNodeIndexes;
	std::vector<std::vector<Checkpoint>> pathCheckpoints;
	bool overlap = false;
	for (Path& path : m_checkpointPaths)
	{
		pathCheckpoints.push_back(path.GeneratePath(checkpointIndex, m_quadblocks, overlap));
		checkpointIndex += pathCheckpoints.back().size();
		linkNodeIndexes.push_back(path.GetStart());
		linkNodeIndexes.push_back(path.GetEnd());
	}
	m_checkpoints.clear();
	for (const std::vector<Checkpoint>& checkpoints : pathCheckpoints)
	{
		for (const Checkpoint& checkpoint : checkpoints)
		{
			m_checkpoints.push_back(checkpoint);
		}
	}

	int lastPathIndex = static_cast<int>(m_checkpointPaths.size()) - 1;
	const Checkpoint* currStartCheckpoint = &m_checkpoints[0];
	float distFinish = 0.0f;
	for (int i = lastPathIndex; i >= 0; i--)
	{
		m_checkpointPaths[i].UpdateDist(distFinish, currStartCheckpoint->GetPos(), m_checkpoints);
		currStartCheckpoint = &m_checkpoints[m_checkpointPaths[i].GetStart()];
		distFinish = currStartCheckpoint->GetDistFinish();
	}

	for (size_t i = 0; i < linkNodeIndexes.size(); i++)
	{
		Checkpoint& node = m_checkpoints[linkNodeIndexes[i]];
		if (i % 2 == 0)
		{
			size_t linkDown = (i == 0) ? linkNodeIndexes.size() - 1 : i - 1;
			node.UpdateDown(static_cast<int>(linkNodeIndexes[linkDown]));
		}
		else
		{
			size_t linkUp = (i + 1) % linkNodeIndexes.size();
			node.UpdateUp(static_cast<int>(linkNodeIndexes[linkUp]));
		}
	}

	for (Path& path : m_checkpointPaths)
	{
		const Checkpoint& middleStart = m_checkpoints[path.GetStart()];
		const Checkpoint& middleEnd = m_checkpoints[path.GetEnd()];

		Path* sides[2] = { path.GetLeft(), path.GetRight() };
		for (Path* side : sides)
		{
			if (!side) { continue; }
			m_checkpoints[side->GetStart()].UpdateDown(middleStart.GetDown());
			m_checkpoints[side->GetEnd()].UpdateUp(middleEnd.GetUp());
		}
	}

	// Cap the number of checkpoints to 255
	const size_t MAX_CHECKPOINTS = 255;
	if (m_checkpoints.size() > MAX_CHECKPOINTS)
	{
		std::unordered_set<size_t> protectedIndices(linkNodeIndexes.begin(), linkNodeIndexes.end());
		for (size_t i = 0; i < m_checkpoints.size(); ++i)
		{
			const Checkpoint& cp = m_checkpoints[i];
			if (cp.GetRight() != NONE_CHECKPOINT_INDEX || cp.GetLeft() != NONE_CHECKPOINT_INDEX)
			{
				protectedIndices.insert(i);
			}
		}

		// Build dist->index map for candidates
		std::multimap<float, size_t> distToNextMap;
		std::unordered_map<size_t, float> currentDistances;

		for (size_t i = 0; i < m_checkpoints.size(); ++i)
		{
			if (protectedIndices.find(i) != protectedIndices.end()) continue;
			const Checkpoint& cp = m_checkpoints[i];
			int downIndex = cp.GetDown();
			if (downIndex != NONE_CHECKPOINT_INDEX)
			{
				float distToNext = (m_checkpoints[downIndex].GetPos() - cp.GetPos()).Length();
				currentDistances[i] = distToNext;
				distToNextMap.insert({distToNext, i});
			}
		}

		size_t total = m_checkpoints.size();
		size_t numToRemove = total - MAX_CHECKPOINTS;
		size_t numRemovable = distToNextMap.size();
		if (numToRemove > numRemovable)
		{
			numToRemove = numRemovable;
		}

		// Heuristic: Pick smallest-dist-to-next checkpoint to remove
		std::unordered_set<size_t> indexesToRemove;
		auto it = distToNextMap.begin();
		for (size_t i = 0; i < numToRemove && it != distToNextMap.end(); )
		{
			size_t candidateIndex = it->second;
			indexesToRemove.insert(candidateIndex);

			// Update distance for previous checkpoint
			const Checkpoint& removedCP = m_checkpoints[candidateIndex];
			int upIndex = removedCP.GetUp();
			int downIndex = removedCP.GetDown();

			if (upIndex != NONE_CHECKPOINT_INDEX &&
				downIndex != NONE_CHECKPOINT_INDEX &&
				protectedIndices.find(upIndex) == protectedIndices.end())
			{
				// Update the distance for the checkpoint before this one
				float oldDist = currentDistances[upIndex];
				float removedDist = currentDistances[candidateIndex];
				float newDist = oldDist + removedDist;

				auto range = distToNextMap.equal_range(oldDist);
				for (auto mapIt = range.first; mapIt != range.second; ++mapIt)
				{
					if (mapIt->second == upIndex)
					{
						distToNextMap.erase(mapIt);
						break;
					}
				}

				currentDistances[upIndex] = newDist;
				distToNextMap.insert({newDist, upIndex});
			}

			++it;
			++i;
		}

		// Build mapping oldIndex -> newIndex
		std::vector<int> oldToNew(total, -1);
		std::vector<Checkpoint> newCheckpoints;
		newCheckpoints.reserve(MAX_CHECKPOINTS);

		for (size_t old = 0; old < total; ++old)
		{
			if (indexesToRemove.find(old) == indexesToRemove.end())
			{
				int newIdx = static_cast<int>(newCheckpoints.size());
				oldToNew[old] = newIdx;
				// copy original checkpoint
				newCheckpoints.push_back(m_checkpoints[old]);
			}
		}

		// Update links
		const int N = static_cast<int>(newCheckpoints.size());

		for (int i = 0; i < N; ++i)
		{
			newCheckpoints[i].SetIndex(i);

			int newUp = (i + 1) % N;
			int newDown = (i == 0) ? (N - 1) : (i - 1);
			newCheckpoints[i].UpdateUp(newUp);
			newCheckpoints[i].UpdateDown(newDown);
		}

		// Update quadblock checkpoint references
		for (Quadblock& qb : m_quadblocks)
		{
			int oldCheckpoint = qb.GetCheckpoint();
			if (oldCheckpoint >= 0 && oldCheckpoint < static_cast<int>(oldToNew.size()))
			{
				int newCheckpoint = oldToNew[oldCheckpoint];
				if (newCheckpoint == -1)
				{
					// This checkpoint was removed, find nearest valid checkpoint
					float minDist = std::numeric_limits<float>::max();
					int nearestCheckpoint = 0;
					Vec3 qbCenter = qb.GetBoundingBox().Midpoint();

					for (int i = 0; i < N; ++i)
					{
						float dist = (newCheckpoints[i].GetPos() - qbCenter).Length();
						if (dist < minDist)
						{
							minDist = dist;
							nearestCheckpoint = i;
						}
					}
					qb.SetCheckpoint(nearestCheckpoint);
				}
				else
				{
					qb.SetCheckpoint(newCheckpoint);
				}
			}
		}

		m_checkpoints = std::move(newCheckpoints);
	}

	UpdateRenderCheckpointData();
	return !overlap;
}


enum class PresetHeader : unsigned
{
	SPAWN, LEVEL, PATH, MATERIAL, TURBO_PAD, ANIM_TEXTURES, SCRIPT
};

bool Level::LoadPreset(const std::filesystem::path& filename)
{
	m_showLogWindow = true;
	nlohmann::json json = nlohmann::json::parse(std::ifstream(filename));
	if (!json.contains("header"))
	{
		m_logMessage += "\nFailed loaded preset: " + filename.string();
		return false;
	}

	const PresetHeader header = json["header"];
	if (header == PresetHeader::SPAWN)
	{
		if (json.contains("spawn")) { m_spawn = json["spawn"]; }
	}
	else if (header == PresetHeader::LEVEL)
	{
		if (json.contains("configFlags")) { m_configFlags = json["configFlags"]; }
		if (json.contains("jumpYSpeedCap")) { m_jumpYSpeedCap = json["jumpYSpeedCap"]; }
		if (json.contains("skyGradient")) { m_skyGradient = json["skyGradient"]; }
		if (json.contains("clearColor")) { m_clearColor = json["clearColor"]; }
		if (json.contains("stars")) { json["stars"].get_to(m_stars); }
		if (json.contains("splitLines"))
		{
			m_splitLines[0] = json["splitLines"][0];
			m_splitLines[1] = json["splitLines"][1];
		}
		if (json.contains("skyboxObjPath"))
		{
			std::string skyboxPath = json["skyboxObjPath"];
			if (!skyboxPath.empty())
			{
				if (m_skybox.LoadOBJ(skyboxPath))
				{
					GenerateRenderSkyboxData();
				}
			}
		}
	}
	else if (header == PresetHeader::PATH)
	{
		if (json.contains("pathCount"))
		{
			const size_t pathCount = json["pathCount"];
			m_checkpointPaths.resize(pathCount);
			for (size_t i = 0; i < pathCount; i++)
			{
				if (!json.contains("path" + std::to_string(i))) { continue; }

				nlohmann::json& pathJson = json["path" + std::to_string(i)];
				if (!pathJson.contains("index")) { continue; }

				size_t index = pathJson["index"];
				Path& path = m_checkpointPaths[index];
				path.FromJson(pathJson, m_quadblocks);
			}
			GenerateCheckpoints();
		}
	}
	else if (header == PresetHeader::MATERIAL)
	{
		if (json.contains("materials"))
		{
			std::vector<std::string> materials = json["materials"];
			for (const std::string& material : materials)
			{
				if (m_materialToQuadblocks.contains(material))
				{
					if (json.contains(material + "_terrain"))
					{
						m_propTerrain.SetPreview(material, json[material + "_terrain"]);
						m_propTerrain.Apply(material, m_materialToQuadblocks[material], m_quadblocks);
					}
					if (json.contains(material + "_quadflags"))
					{
						m_propQuadFlags.SetPreview(material, json[material + "_quadflags"]);
						m_propQuadFlags.Apply(material, m_materialToQuadblocks[material], m_quadblocks);
					}
					if (json.contains(material + "_drawflags"))
					{
						m_propDoubleSided.SetPreview(material, json[material + "_drawflags"]);
						m_propDoubleSided.Apply(material, m_materialToQuadblocks[material], m_quadblocks);
					}
					if (json.contains(material + "_checkpoint"))
					{
						m_propCheckpoints.SetPreview(material, json[material + "_checkpoint"]);
						m_propCheckpoints.Apply(material, m_materialToQuadblocks[material], m_quadblocks);
					}
					if (json.contains(material + "_trigger"))
					{
						QuadblockTrigger trigger = json[material + "_trigger"];
						m_propTurboPads.GetBackup(material) = trigger;
						m_propTurboPads.GetPreview(material) = trigger;
					}
					if (json.contains(material + "_speedImpact"))
					{
						m_propSpeedImpact.SetPreview(material, json[material + "_speedImpact"]);
						m_propSpeedImpact.Apply(material, m_materialToQuadblocks[material], m_quadblocks);
					}
					if (json.contains(material + "_checkpointPathable"))
					{
						m_propCheckpointPathable.SetPreview(material, json[material + "_checkpointPathable"]);
						m_propCheckpointPathable.Apply(material, m_materialToQuadblocks[material], m_quadblocks);
					}
					if (json.contains(material + "_visTreeTransparent"))
					{
						m_propVisTreeTransparent.SetPreview(material, json[material + "_visTreeTransparent"]);
						m_propVisTreeTransparent.Apply(material, m_materialToQuadblocks[material], m_quadblocks);
					}
					if (json.contains(material + "_drawOrderHigh"))
					{
						m_propDrawOrderHigh.SetPreview(material, json[material + "_drawOrderHigh"]);
						m_propDrawOrderHigh.Apply(material, m_materialToQuadblocks[material], m_quadblocks);
					}
				}
			}
		}
	}
	else if (header == PresetHeader::ANIM_TEXTURES)
	{
		if (json.contains("animCount"))
		{
			const size_t animCount = json["animCount"];
			for (size_t i = 0; i < animCount; i++)
			{
				if (!json.contains("anim" + std::to_string(i))) { continue; }
				AnimTexture animTexture;
				animTexture.FromJson(json["anim" + std::to_string(i)], m_quadblocks, m_parentPath);
				if (animTexture.IsPopulated()) { m_animTextures.push_back(animTexture); }
			}
		}
	}
	else if (header == PresetHeader::TURBO_PAD)
	{
		if (json.contains("turbopads"))
		{
			std::unordered_set<std::string> turboPads = json["turbopads"];
			for (Quadblock& quadblock : m_quadblocks)
			{
				const std::string& quadName = quadblock.GetName();
				if (turboPads.contains(quadName))
				{
					if (!json.contains(quadName + "_trigger")) { continue; }
					quadblock.SetTrigger(json[quadName + "_trigger"]);
					ManageTurbopad(quadblock);
					if (m_bsp.IsValid())
					{
						m_bsp.Clear();
						GenerateRenderBspData();
					}
				}
			}
		}
	}
	else if (header == PresetHeader::SCRIPT)
	{
		m_pythonScript = json["script"];
	}
	else
	{
		m_logMessage += "\nFailed loaded preset: " + filename.string();
		return false;
	}
	m_logMessage += "\nSuccessfully loaded preset: " + filename.string();
	return true;
}

bool Level::SavePreset(const std::filesystem::path& path)
{
	std::filesystem::path dirPath = path / (m_name + "_presets");
	if (!std::filesystem::exists(dirPath)) { std::filesystem::create_directory(dirPath); }

	auto SaveJSON = [](const std::filesystem::path& path, const nlohmann::json& json)
		{
			std::ofstream pathFile(path);
			pathFile << std::setw(4) << json << std::endl;
		};

	nlohmann::json spawnJson = {};
	spawnJson["header"] = PresetHeader::SPAWN;
	spawnJson["spawn"] = m_spawn;
	SaveJSON(dirPath / "spawn.json", spawnJson);

	nlohmann::json levelJson = {};
	levelJson["header"] = PresetHeader::LEVEL;
	levelJson["configFlags"] = m_configFlags;
	levelJson["skyGradient"] = m_skyGradient;
	levelJson["clearColor"] = m_clearColor;
	levelJson["stars"] = m_stars;
	levelJson["jumpYSpeedCap"] = m_jumpYSpeedCap;
	levelJson["splitLines"] = { m_splitLines[0], m_splitLines[1] };
	if (!m_skybox.m_objPath.empty()) { levelJson["skyboxObjPath"] = m_skybox.m_objPath.string(); }
	SaveJSON(dirPath / "level.json", levelJson);

	nlohmann::json pathJson = {};
	pathJson["header"] = PresetHeader::PATH;
	pathJson["pathCount"] = m_checkpointPaths.size();
	for (size_t i = 0; i < m_checkpointPaths.size(); i++)
	{
		pathJson["path" + std::to_string(i)] = nlohmann::json();
		m_checkpointPaths[i].ToJson(pathJson["path" + std::to_string(i)], m_quadblocks);
	}
	SaveJSON(dirPath / "path.json", pathJson);

	if (!m_materialToQuadblocks.empty())
	{
		nlohmann::json materialJson = {};
		materialJson["header"] = PresetHeader::MATERIAL;
		std::vector<std::string> materials; materials.reserve(m_materialToQuadblocks.size());
		for (const auto& [key, value] : m_materialToQuadblocks)
		{
			materials.push_back(key);
			materialJson[key + "_terrain"] = m_propTerrain.GetBackup(key);
			materialJson[key + "_quadflags"] = m_propQuadFlags.GetBackup(key);
			materialJson[key + "_drawflags"] = m_propDoubleSided.GetBackup(key);
			materialJson[key + "_checkpoint"] = m_propCheckpoints.GetBackup(key);
			materialJson[key + "_checkpointPathable"] = m_propCheckpointPathable.GetBackup(key);
			materialJson[key + "_visTreeTransparent"] = m_propVisTreeTransparent.GetBackup(key);
			materialJson[key + "_trigger"] = m_propTurboPads.GetBackup(key);
			materialJson[key + "_speedImpact"] = m_propSpeedImpact.GetBackup(key);
			materialJson[key + "_drawOrderHigh"] = m_propDrawOrderHigh.GetBackup(key);
		}
		materialJson["materials"] = materials;
		SaveJSON(dirPath / "material.json", materialJson);
	}

	if (!m_animTextures.empty())
	{
		nlohmann::json animJson = {};
		animJson["header"] = PresetHeader::ANIM_TEXTURES;
		animJson["animCount"] = m_animTextures.size();
		size_t i = 0;
		for (const AnimTexture& animTexture : m_animTextures)
		{
			animTexture.ToJson(animJson["anim" + std::to_string(i++)], m_quadblocks);
		}
		SaveJSON(dirPath / "animtex.json", animJson);
	}

	std::unordered_set<std::string> turboPads;
	nlohmann::json turboPadJson = {};
	for (const Quadblock& quadblock : m_quadblocks)
	{
		if (quadblock.GetTurboPadIndex() == TURBO_PAD_INDEX_NONE) { continue; }
		const std::string& quadName = quadblock.GetName();
		turboPads.insert(quadName);
		turboPadJson[quadName + "_trigger"] = quadblock.GetTrigger();
	}
	if (!turboPads.empty())
	{
		turboPadJson["header"] = PresetHeader::TURBO_PAD;
		turboPadJson["turbopads"] = turboPads;
		SaveJSON(dirPath / "turbopad.json", turboPadJson);
	}

	if (m_saveScript)
	{
		nlohmann::json scriptJson = {};
		scriptJson["header"] = PresetHeader::SCRIPT;
		scriptJson["script"] = m_pythonScript;
		SaveJSON(dirPath / "script.json", scriptJson);
	}
	return true;
}

void Level::ResetFilter()
{
	for (Quadblock& qb : m_quadblocks)
	{
		qb.SetFilter(false);
		qb.SetFilterColor(GuiRenderSettings::defaultFilterColor);
	}
}

void Level::ResetRendererSelection()
{
	m_rendererQueryPoint = Vec3();
	m_rendererSelectedQuadblockIndexes.clear();
	m_models[LevelModels::SELECTED]->GetMesh().Clear();
}

void Level::ManageTurbopad(Quadblock& quadblock)
{
	bool stp = true;
	size_t turboPadIndex = TURBO_PAD_INDEX_NONE;
	switch (quadblock.GetTrigger())
	{
	case QuadblockTrigger::TURBO_PAD:
		stp = false;
	case QuadblockTrigger::SUPER_TURBO_PAD:
	{
		Quadblock turboPad = quadblock;
		const Vec3 up(0.0f, 1.0f, 0.0f);
		turboPad.Translate(TURBO_PAD_QUADBLOCK_TRANSLATION, up);
		turboPad.SetCheckpoint(-1);
		turboPad.SetCheckpointStatus(false);
		turboPad.SetVisTreeTransparent(false);
		turboPad.SetName(quadblock.GetName() + (stp ? "_stp" : "_tp"));
		turboPad.SetFlag(QuadFlags::TRIGGER_SCRIPT | QuadFlags::INVISIBLE_TRIGGER | QuadFlags::WALL);
		turboPad.SetTerrain(stp ? TerrainType::SUPER_TURBO_PAD : TerrainType::TURBO_PAD);
		turboPad.SetTurboPadIndex(TURBO_PAD_INDEX_NONE);
		turboPad.SetHide(true);
		turboPad.SetAnimated(false);
		turboPad.SetDrawOrderHigh(0);

		size_t index = m_quadblocks.size();
		turboPadIndex = quadblock.GetTurboPadIndex();
		quadblock.SetTurboPadIndex(index);
		m_quadblocks.push_back(turboPad);
		if (turboPadIndex == TURBO_PAD_INDEX_NONE) { break; }
	}
	case QuadblockTrigger::NONE:
	{
		bool clearTurboPadIndex = false;
		if (turboPadIndex == TURBO_PAD_INDEX_NONE)
		{
			clearTurboPadIndex = true;
			turboPadIndex = quadblock.GetTurboPadIndex();
		}
		if (turboPadIndex == TURBO_PAD_INDEX_NONE) { break; }

		for (Quadblock& quad : m_quadblocks)
		{
			size_t index = quad.GetTurboPadIndex();
			if (index > turboPadIndex) { quad.SetTurboPadIndex(index - 1); }
		}

		if (clearTurboPadIndex) { quadblock.SetTurboPadIndex(TURBO_PAD_INDEX_NONE); }
		m_quadblocks.erase(m_quadblocks.begin() + turboPadIndex);
		break;
	}
	}
}


// Build a .ctrmodel binary from raw LEV data and save it to outputPath
// Returns true on success
bool Level::LoadLEV(const std::filesystem::path& levFile)
{
	std::ifstream file(levFile, std::ios::binary);
	if (!file.is_open()) return false;

	// Read entire LEV file into memory for direct model access
	file.seekg(0, std::ios::end);
	size_t levSize = file.tellg();
	file.seekg(0, std::ios::beg);
	m_levData.resize(levSize);
	file.read(reinterpret_cast<char*>(m_levData.data()), levSize);

	// Read VRM for model texture extraction
	{
		std::filesystem::path vrmPath = levFile;
		vrmPath.replace_extension(".vrm");
		m_vramData = ReadRawVRAM(vrmPath);
	}

	// Reset file position for existing loading logic
	file.seekg(0, std::ios::beg);

	m_hasRawTexture = true;

	m_parentPath = levFile.parent_path();
	m_name = levFile.filename().replace_extension().string() + "_edit";

	uint32_t offPointerMap;
	Read(file, offPointerMap);

	std::streampos offLev = file.tellg();

	std::set<uint32_t> pointerMap;
	file.seekg(offLev + std::streampos(offPointerMap));
	uint32_t pointerMapSize;
	Read(file, pointerMapSize);
	for (size_t i = 0; i < pointerMapSize / sizeof(uint32_t); i++)
	{
		uint32_t pointer;
		Read(file, pointer);
		pointerMap.insert(pointer);
	}

	file.seekg(offLev);
	PSX::LevHeader header = {};
	Read(file, header);

	m_configFlags = header.config;
	m_clearColor = ConvertColor(header.clear);
	m_stars = ConvertStars(header.stars);
	m_jumpYSpeedCap = static_cast<int>(header.jumpYSpeedCap);
	m_splitLines[0] = ConvertFP(header.splitLines[0], FP_ONE_GEO);
	m_splitLines[1] = ConvertFP(header.splitLines[1], FP_ONE_GEO);
	for (size_t i = 0; i < m_spawn.size(); i++)
	{
		m_spawn[i].pos = ConvertPSXVec3(header.driverSpawn[i].pos, FP_ONE_GEO);
		m_spawn[i].rot = ConvertPSXAngle(header.driverSpawn[i].rot);
	}
	for (size_t i = 0; i < NUM_GRADIENT; i++)
	{
		m_skyGradient[i].posFrom = ConvertFP(header.skyGradient[i].posFrom, 1u);
		m_skyGradient[i].posTo = ConvertFP(header.skyGradient[i].posTo, 1u);
		m_skyGradient[i].colorFrom = ConvertColor(header.skyGradient[i].colorFrom);
		m_skyGradient[i].colorTo = ConvertColor(header.skyGradient[i].colorTo);
	}

	PSX::MeshInfo meshInfo = {};
	file.seekg(offLev + std::streampos(header.offMeshInfo));
	Read(file, meshInfo);

	std::vector<PSX::Vertex> vertices;
	vertices.reserve(meshInfo.numVertices);
	file.seekg(offLev + std::streampos(meshInfo.offVertices));
	for (uint32_t i = 0; i < meshInfo.numVertices; i++)
	{
		PSX::Vertex vertex = {};
		Read(file, vertex);
		vertices.push_back(vertex);
	}


	// Loading textures and animated textures and quadblocks
	std::filesystem::path vrmPath = levFile;
	vrmPath.replace_extension(".vrm");
	std::vector<uint16_t> vram =  ReadRawVRAM(vrmPath);
	int texCounter = 0;
	std::vector<uint32_t> quadblocksVisibleSetOff; // List of VisibleSetOffset for quadblock. Needed for vistree loading, parsed with quadblocks.
	std::unordered_map<LayoutKey, PixelBounds> textureToPixelBounds; // Map Layout key -> Pixels bounds of the texture.
	std::unordered_map<LayoutKey, std::string> materialCache; // Layout Key -> matName
	std::map<size_t, std::map<size_t, uint32_t>> quadblockFaceToAnimOffset; // Map: quadblock index -> face index -> AnimTex offset
	std::unordered_map<uint32_t, std::string> textureGroupToMaterial; // Map : texture group offset -> material name
	m_rawAnimTex.clear(); // Map : Absolute Offset -> PSX::AnimTex
	m_rawTextureGroup.clear(); // Map : Absolute Offset ->  PSX::TextureGroup
	m_rawAnimTexFrames.clear(); // Map : Absolute Offset -> List of Absolute Offset for PSX::TextureGroup

	std::filesystem::path tempDir = levFile.parent_path() / (levFile.stem().string() + "_textures");
	std::filesystem::create_directories(tempDir);

	bool hasAnimData = header.offAnimTex > 0;
	size_t offAnimStart = header.offAnimTex;

	
	// 1st pass : Parse Quadblock, find TextureGroups, and caclulate UV bounds
	// Take care of all texture group for static quad and animated quads
	file.seekg(offLev + std::streampos(meshInfo.offQuadblocks));
	for (uint32_t i = 0; i < meshInfo.numQuadblocks; i++)
	{
		PSX::Quadblock psxQuad = {};
		Read(file, psxQuad);
		std::streampos currentPosQuad = file.tellg();
		for (int f = 0; f < NUM_FACES_QUADBLOCK + 1; f++)
		{
			uint32_t texOffset = f == NUM_FACES_QUADBLOCK ? psxQuad.offLowTexture : psxQuad.offMidTextures[f];

			// How to know if a texture is animated or not : POINTERFLAG. ODD = ANIMTEX. EVEN = STATICTEX
			if (hasAnimData && texOffset >= offAnimStart && pointerMap.contains(texOffset - 1)) // Anim Textures
			{
				if (!m_rawAnimTex.contains(texOffset-1))
				{
					file.seekg(offLev + std::streampos(texOffset-1));
					PSX::AnimTex animTex;
					Read(file, animTex);
					m_rawAnimTex[texOffset - 1] = animTex;

					std::vector<uint32_t> frameTextureGroupOffset;
					for (uint16_t frame = 0; frame < animTex.frameCount; frame++)
					{
						uint32_t frameTexOffset;
						Read(file, frameTexOffset);
						frameTextureGroupOffset.push_back(frameTexOffset);

						std::streampos currentPos = file.tellg();
						file.seekg(offLev + static_cast<std::streamoff>(frameTexOffset));
						PSX::TextureGroup group = {};
						Read(file, group);
						file.seekg(currentPos);
						// Tempfix for vanilla : group.mosaic is broken for a lot of texture, need research
						PSX::TextureGroup tempTexGroup = {};
						tempTexGroup.far = group.far;
						tempTexGroup.middle = group.middle;
						tempTexGroup.near = group.near;
						tempTexGroup.mosaic = group.near;
						m_rawTextureGroup[frameTexOffset] = tempTexGroup;
						const PSX::TextureLayout& layout = group.middle;
						LayoutKey key(layout);

						if (!materialCache.contains(key))
						{
							std::string newMatName = "tex_" + std::to_string(texCounter++);
							materialCache[key] = newMatName;
						}
						textureGroupToMaterial[frameTexOffset] = materialCache[key];

						RawUV rawUV(layout);
						textureToPixelBounds[key].Update(rawUV);

					}
					m_rawAnimTexFrames[texOffset - 1] = frameTextureGroupOffset;
				}

				quadblockFaceToAnimOffset[i][f] = texOffset - 1;

			}
			else // Regular Textures
			{
				file.seekg(offLev + static_cast<std::streamoff>(texOffset));
				PSX::TextureGroup group = {};
				Read(file, group);
				// Tempfix for vanilla : group.mosaic is broken for a lot of texture, need research
				PSX::TextureGroup tempTexGroup = {};
				tempTexGroup.far = group.far;
				tempTexGroup.middle = group.middle;
				tempTexGroup.near = group.near;
				tempTexGroup.mosaic = group.near;
				m_rawTextureGroup[texOffset] = tempTexGroup;
				const PSX::TextureLayout& layout = group.middle;
				LayoutKey key(layout);

				if (!materialCache.contains(key))
				{
					std::string newMatName = "tex_" + std::to_string(texCounter++);
					materialCache[key] = newMatName;
				}
				textureGroupToMaterial[texOffset] = materialCache[key];

				RawUV rawUV(layout, psxQuad.drawOrderLow, f);
				textureToPixelBounds[key].Update(rawUV);
			}

		}
		file.seekg(currentPosQuad);
	}

	// 3rd pass : Create PNGs and Materials
	for (const auto& [key, bounds] : textureToPixelBounds)
	{
		std::string newMatName = materialCache[key];
		Texture newTexture(key, bounds, vram, newMatName, tempDir, true);
		m_materialToTexture[newMatName] = newTexture;
	}


	// 4th pass : create quadblocks with material, UVs and texture	
	file.seekg(offLev + std::streampos(meshInfo.offQuadblocks));
	for (uint32_t i = 0; i < meshInfo.numQuadblocks; i++)
	{
		PSX::Quadblock psxQuad = {};
		Read(file, psxQuad);
		quadblocksVisibleSetOff.push_back(psxQuad.offVisibleSet);
		Quadblock& qb = m_quadblocks.emplace_back(psxQuad, vertices, [this](const Quadblock& qb) { UpdateFilterRenderData(qb); });
		bool materialAssigned = false;
		std::string qbMatName = "default";
		for (int f = 0; f < 4; f++) 
		{
			uint32_t texOffset = psxQuad.offMidTextures[f];
			if (hasAnimData && texOffset >= offAnimStart && pointerMap.contains(texOffset - 1)) // Anim Texture
			{
				qb.SetAnimated(true);
			}
			
			else 
			{
				std::streampos currentPos = file.tellg();
				file.seekg(offLev + static_cast<std::streamoff>(texOffset));
				PSX::TextureGroup group = {};
				Read(file, group);
				file.seekg(currentPos);

				const PSX::TextureLayout& layout = group.middle;
				LayoutKey key(layout);

				if (!materialAssigned)
				{
					qbMatName = materialCache[key];
					qb.SetMaterial(qbMatName);
					qb.SetTexPath(m_materialToTexture[qbMatName].GetPath());
					m_materialToQuadblocks[qbMatName].push_back(i);
					materialAssigned = true;
				}

				RawUV rawUV(layout, psxQuad.drawOrderLow, f);
				const PixelBounds& bounds = textureToPixelBounds[key];
				float croppedWidth = static_cast<float>(bounds.maxU - bounds.minU);
				float croppedHeight = static_cast<float>(bounds.maxV - bounds.minV);
				if (croppedWidth == 0) croppedWidth = 1.0f;
				if (croppedHeight == 0) croppedHeight = 1.0f;
				QuadUV uvs = {
					Vec2((rawUV.u0 - bounds.minU) / croppedWidth, (rawUV.v0 - bounds.minV) / croppedHeight),
					Vec2((rawUV.u1 - bounds.minU) / croppedWidth, (rawUV.v1 - bounds.minV) / croppedHeight),
					Vec2((rawUV.u2 - bounds.minU) / croppedWidth, (rawUV.v2 - bounds.minV) / croppedHeight),
					Vec2((rawUV.u3 - bounds.minU) / croppedWidth, (rawUV.v3 - bounds.minV) / croppedHeight)
				};
				qb.SetFaceUVs(f, uvs);
			}
		}
		if (!materialAssigned) 
		{
			qb.SetMaterial("default");
			m_materialToQuadblocks["default"].push_back(i);
		}
	}

	//5th pass : Create .obj for AnimText, and assign to quads
	if (hasAnimData)
	{
		std::map<std::map<size_t, uint32_t>, std::set<size_t>> facePatternToQuadblocks;
		for (const auto& [quadIdx, faceMap] : quadblockFaceToAnimOffset)
		{
			facePatternToQuadblocks[faceMap].insert(quadIdx);
		}

		std::set<std::map<size_t, uint32_t>> processedPatterns;
		for (const auto& [faceMap, quadSet] : facePatternToQuadblocks)
		{
			if (processedPatterns.contains(faceMap)) continue;
			if (faceMap.empty()) continue;

			std::vector<size_t> quadIndices(quadSet.begin(), quadSet.end());

			uint32_t firstAnimOffset = faceMap.begin()->second;
			if (!m_rawAnimTex.contains(firstAnimOffset)) continue;

			const PSX::AnimTex& firstAnimData = m_rawAnimTex[firstAnimOffset];
			size_t frameCount = firstAnimData.frameCount;

			// Verify all AnimTex in this pattern have the same frame count
			bool validAnimation = true;
			for (const auto& [faceIdx, animOffset] : faceMap)
			{
				if (!m_rawAnimTex.contains(animOffset) || !m_rawAnimTexFrames.contains(animOffset) || m_rawAnimTex[animOffset].frameCount != frameCount)
				{
					validAnimation = false;
					break;
				}
			}
			if (!validAnimation) continue;


			std::array<std::vector<PSX::TextureLayout>, 4> faceFrameLayouts;
			std::array<std::vector<std::string>, 4> faceFrameMaterials;

			bool allMaterialsFound = true;

			for (const auto& [faceIdx, animOffset] : faceMap)
			{
				for (uint32_t textureGroupOffset : m_rawAnimTexFrames.at(animOffset))
				{
					if (!textureGroupToMaterial.contains(textureGroupOffset)) 
					{
						allMaterialsFound = false;
						break;
					}
					faceFrameMaterials[faceIdx].push_back(textureGroupToMaterial[textureGroupOffset]);
					std::streampos savedPos = file.tellg();
					file.seekg(offLev + std::streampos(textureGroupOffset));
					PSX::TextureGroup group = {};
					Read(file, group);
					file.seekg(savedPos);
					faceFrameLayouts[faceIdx].push_back(group.middle);
				}
				if (!allMaterialsFound) break;
			}

			if (!allMaterialsFound) continue;

			// Create temporary OBJ file
			std::string animName = "";
			for (size_t faceIdx = 0; faceIdx < 4; faceIdx++)
			{
				if (faceFrameMaterials[faceIdx].size() != 0)
				{
					animName = faceFrameMaterials[faceIdx][0];
					break;
				}
			}

			std::filesystem::path animDir = tempDir / animName;
			std::filesystem::create_directories(animDir);

			AnimTexture animTexture(animName, tempDir, faceFrameLayouts, faceFrameMaterials, quadIndices, m_quadblocks, textureToPixelBounds, m_materialToTexture, firstAnimData, m_animTextures);

			if (!animTexture.IsEmpty())
			{
				animTexture.SetStartFrame(firstAnimData.startAtFrame);
				animTexture.SetDuration(firstAnimData.frameDuration);

				for (size_t quadIdx : quadIndices)
				{
					animTexture.AddQuadblockIndex(quadIdx);
					std::string oldMat = m_quadblocks[quadIdx].GetMaterial();
					auto& v = m_materialToQuadblocks[oldMat];
					v.erase(std::remove(v.begin(), v.end(), quadIdx), v.end());
					m_quadblocks[quadIdx].SetMaterial(animName);
					m_materialToQuadblocks[animName].push_back(quadIdx);
				}
				m_animTextures.push_back(animTexture);
				processedPatterns.insert(faceMap);
			}
			else
			{
				printf("WARNING : Empty animtex\n");
			}
		}
	}

	m_bsp.Clear();
	file.seekg(offLev + std::streampos(meshInfo.offBSPNodes));
	std::vector<BSP*> bspArray;
	for (uint32_t i = 0; i < meshInfo.numBSPNodes; i++)
	{
		bspArray.push_back(new BSP());
	}

	for (uint32_t i = 0; i < meshInfo.numBSPNodes; i++)
	{
		uint16_t flag;
		std::streampos nodeStart = file.tellg();
		Read(file, flag);
		file.seekg(nodeStart);

		if (flag & BSPFlags::LEAF)
		{
			PSX::BSPLeaf leaf = {};
			Read(file, leaf);
			bspArray[leaf.id]->PopulateLeaf(leaf, bspArray, m_quadblocks, meshInfo.offQuadblocks, meshInfo.numBSPNodes);
		}
		else
		{
			PSX::BSPBranch branch = {};
			Read(file, branch);
			if (branch.unk2 != 0 || branch.unk3 != 0) { printf("Branch ID%d has child 3\n", branch.id); }
			bspArray[branch.id]->PopulateBranch(branch, bspArray, meshInfo.numBSPNodes);
		}
	}

	if (!bspArray.empty())
	{
		m_bsp = *(bspArray[0]);
		m_bsp.PopulateBranchQuadIndexes();
		if (m_bsp.IsValid()) { GenerateRenderBspData(); }
		else { m_bsp.Clear(); }
	}
	else { m_bsp.Clear(); }
	std::set<size_t> validID;
	
	printf("BSP ARRAY SIZE : %d\n", bspArray.size());
	std::vector<const BSP*> tree = m_bsp.GetTree();
	printf("BSP TREE SIZE : %d\n", tree.size());
	for (const BSP* bsp : tree) { validID.insert(bsp->GetId()); }
	for (BSP* bsp : bspArray) { if (!validID.contains(bsp->GetId())) { printf("ID %d isn't in tree\n", bsp->GetId()); } }
	


	// Load VisTree
	if (header.offVisMem != 0)
	{
		file.seekg(offLev + static_cast<std::streamoff>(header.offVisMem));
		PSX::VisualMem visMem = {};
		Read(file, visMem);

		if (visMem.offNodes[0] != 0)
		{
			std::vector<const BSP*> bspLeaves = m_bsp.GetLeaves();
			std::vector<const BSP*> bspNodes = m_bsp.GetTree();

			m_bspVis = BitMatrix(bspLeaves.size(), bspLeaves.size());

			std::map<size_t, size_t> leafIdToMatrix;
			for (size_t i = 0; i < bspLeaves.size(); i++)
			{
				leafIdToMatrix[bspLeaves[i]->GetId()] = i;
			}

			const size_t visNodeSize = (bspNodes.size() + 31) / 32;

			auto decompressVisNodes = [&](std::streampos srcPos) -> std::vector<uint32_t>
				{
					std::vector<uint8_t> dst(visNodeSize * sizeof(uint32_t), 0);
					file.seekg(srcPos);
					size_t dstIdx = 0;
					while (dstIdx < dst.size())
					{
						int8_t c;
						Read(file, c);
						if (c == 0) { break; }
						if (c < 0)
						{
							int count = (-c) + 1;
							uint8_t val;
							Read(file, val);
							for (int i = 0; i < count && dstIdx < dst.size(); i++)
								dst[dstIdx++] = val;
						}
						else
						{
							int count = c;
							for (int i = 0; i < count && dstIdx < dst.size(); i++)
							{
								uint8_t val;
								Read(file, val);
								dst[dstIdx++] = val;
							}
						}
					}
					std::vector<uint32_t> result(visNodeSize);
					std::memcpy(result.data(), dst.data(), dst.size());
					return result;
				};

			for (size_t q = 0; q < m_quadblocks.size(); q++)
			{
				uint32_t offVisibleSet = quadblocksVisibleSetOff[q];
				if (offVisibleSet == 0) { continue; }
				PSX::VisibleSet visSet = {};
				file.seekg(offLev + static_cast<std::streamoff>(offVisibleSet));
				Read(file, visSet);
				if (visSet.offVisibleBSPNodes == 0) { continue; }
				size_t leafID = m_quadblocks[q].GetBSPID() & ~BSPID::LEAF;
				if (!leafIdToMatrix.contains(leafID)) { continue; }
				size_t visTreeID = leafIdToMatrix[leafID];

				bool compressed = visSet.offVisibleBSPNodes & 1;
				uint32_t actualOff = visSet.offVisibleBSPNodes & ~3u;
				std::streampos srcPos = offLev + static_cast<std::streamoff>(actualOff);

				std::vector<uint32_t> visNodes;
				if (compressed)
				{
					visNodes = decompressVisNodes(srcPos);
				}
				else
				{
					file.seekg(srcPos);
					visNodes.resize(visNodeSize);
					for (size_t i = 0; i < visNodeSize; i++) { Read(file, visNodes[i]); }
				}

				for (size_t i = 0; i < bspLeaves.size(); i++)
				{
					size_t destBspId = bspLeaves[i]->GetId();
					if (destBspId / 32 >= visNodes.size()) { continue; }
					uint32_t word = visNodes[destBspId / 32];
					uint32_t bit = 1u << (31 - (destBspId % 32));
					if (word & bit) { m_bspVis.Set(true, visTreeID, i); }
				}
			}
		}
		int count = 0;
		for (size_t x = 0; x < m_bspVis.GetHeight(); x++)
		{
			for (size_t y = 0; y < m_bspVis.GetWidth(); y++)
			{
				if (m_bspVis.Get(x, y))
					count++;
			}
		}
		int max = static_cast<int>(m_bspVis.GetHeight() * m_bspVis.GetWidth());
		float ratio = 100.0f * static_cast<float>(count) / static_cast<float>(max);
		printf("Visibility: %d/%d,  %f%%\n", count, max, ratio);
	}

	file.seekg(offLev + std::streampos(header.offCheckpointNodes));
	for (uint32_t i = 0; i < header.numCheckpointNodes; i++)
	{
		PSX::Checkpoint checkpoint = {};
		Read(file, checkpoint);
		m_checkpoints.emplace_back(checkpoint, static_cast<int>(i));
	}
	UpdateRenderCheckpointData();

	// Ghost checkpoint reading
	file.seekg(offLev + std::streampos(header.offCheckpointNodes) + static_cast<std::streamoff>(255 * sizeof(PSX::Checkpoint)));
	PSX::Checkpoint checkpoint255 = {};
	Read(file, checkpoint255);
	file.seekg(offLev + std::streampos(header.offCheckpointNodes) + static_cast<std::streamoff>(checkpoint255.linkUp * sizeof(PSX::Checkpoint)));
	PSX::Checkpoint checkpoint255Next = {};
	Read(file, checkpoint255Next);
	file.seekg(offLev + std::streampos(header.offCheckpointNodes) + static_cast<std::streamoff>(checkpoint255Next.linkUp * sizeof(PSX::Checkpoint)));
	PSX::Checkpoint checkpoint255NextNext = {};
	Read(file, checkpoint255NextNext);
	printf("Checkpoint 255's next : %d\n Checkpoint 255's next 's next : %d\n", checkpoint255.linkUp, checkpoint255Next.linkUp);






	m_tropyGhost.clear();
	m_oxideGhost.clear();
	if (header.offExtra > 0)
	{
		file.seekg(offLev + std::streampos(header.offExtra));
		PSX::LevelExtraHeader extraHeader = {};
		Read(file, extraHeader);
		// Read N. Tropy Ghost
		if (extraHeader.count >= PSX::LevelExtra::N_TROPY_GHOST + 1 &&
			extraHeader.offsets[PSX::LevelExtra::N_TROPY_GHOST] > 0)
		{
			file.seekg(offLev + std::streampos(extraHeader.offsets[PSX::LevelExtra::N_TROPY_GHOST]));
			size_t ghostSize = 0;
			if (extraHeader.count > PSX::LevelExtra::N_OXIDE_GHOST && extraHeader.offsets[PSX::LevelExtra::N_OXIDE_GHOST] > 0)
			{
				ghostSize = extraHeader.offsets[PSX::LevelExtra::N_OXIDE_GHOST] - extraHeader.offsets[PSX::LevelExtra::N_TROPY_GHOST];
			}
			else
			{
				ghostSize = header.offLevNavTable - extraHeader.offsets[PSX::LevelExtra::N_TROPY_GHOST];
			}
			m_tropyGhost.resize(ghostSize);
			file.read(reinterpret_cast<char*>(m_tropyGhost.data()), ghostSize);
		}
		// Read N. Oxide Ghost
		if (extraHeader.count >= PSX::LevelExtra::N_OXIDE_GHOST + 1 && extraHeader.offsets[PSX::LevelExtra::N_OXIDE_GHOST] > 0)
		{
			file.seekg(offLev + std::streampos(extraHeader.offsets[PSX::LevelExtra::N_OXIDE_GHOST]));
			size_t ghostSize = header.offLevNavTable - extraHeader.offsets[PSX::LevelExtra::N_OXIDE_GHOST];
			m_oxideGhost.resize(ghostSize);
			file.read(reinterpret_cast<char*>(m_oxideGhost.data()), ghostSize);
		}
	}

	//Load Skybox
	if (header.offSkybox != 0)
	{
		PSX::Skybox psxSkybox = {};
		file.seekg(offLev + std::streampos(header.offSkybox));
		Read(file, psxSkybox);

		std::vector<PSX::SkyboxVertex> psxVerts(psxSkybox.numVertex);
		file.seekg(offLev + std::streampos(psxSkybox.offVertex));
		for (uint32_t i = 0; i < psxSkybox.numVertex; i++)
		{
			Read(file, psxVerts[i]);
		}

		std::vector<std::vector<uint16_t>> segmentIndices(PSX::NUM_SKYBOX_SEGMENTS);
		for (size_t seg = 0; seg < PSX::NUM_SKYBOX_SEGMENTS; seg++)
		{
			const int16_t faceCount = psxSkybox.numFaces[seg];
			if (faceCount <= 0 || psxSkybox.offFaces[seg] == 0) { continue; }

			const size_t indexCount = static_cast<size_t>(faceCount) * PSX::SKYBOX_FACE_STRIDE;
			segmentIndices[seg].resize(indexCount);
			file.seekg(offLev + std::streampos(psxSkybox.offFaces[seg]));
			for (size_t i = 0; i < indexCount; i++)
			{
				Read(file, segmentIndices[seg][i]);
			}
		}

		std::filesystem::path objPath = m_parentPath / (levFile.filename().replace_extension().string() + "_skybox.obj");
		if (m_skybox.LoadFromPSX(psxSkybox, psxVerts, segmentIndices, objPath)) 
		{
			GenerateRenderSkyboxData();
		}
	}

	if (header.offLevNavTable != 0)
	{
		//printf("off Lev Nav Table : 0x%x\n", header.offLevNavTable);
		file.seekg(offLev + std::streampos(header.offLevNavTable));
		PSX::levAINavTable navTable{};
		Read(file, navTable);
		for (int i = 0; i < 3; i++)
		{
			if (navTable.offAIPathArray[i] != 0)
			{
				//printf("off AI Path Array %d : 0x%x\n", i, navTable.offAIPathArray[i]);
				file.seekg(offLev + std::streampos(navTable.offAIPathArray[i]));
				PSX::NavHeader navHeader{};
				Read(file, navHeader);

				std::vector<PSX::NavFrame> nodes;
				PSX::NavFrame startLine{};
				Read(file, startLine);
				nodes.push_back(startLine);
				for (int j = 0; j < navHeader.numPoints; j++)
				{
					PSX::NavFrame navFrame{};
					Read(file, navFrame);
					nodes.push_back(navFrame);
					//printf("Pos %d : x=%d, y=%d, z=%d\n", j, navFrame.pos.x, navFrame.pos.y, navFrame.pos.z);
					//printf("Rot %d : %d, %d, %d, %d\n", j, navFrame.rot[0], navFrame.rot[1], navFrame.rot[2], navFrame.rot[3]);
				}
				m_botPaths[i] = BotPath(navHeader, nodes);
			}
		}
		UpdateRenderBotData();
	}

	//Load preset models
	std::filesystem::path folderPath(Settings::m_lastOpenedModelFolder);
	if (std::filesystem::exists(folderPath) && std::filesystem::is_directory(folderPath))
	{
		for (const auto& entry : std::filesystem::directory_iterator(folderPath))
		{
			if (!entry.is_regular_file())
				continue;

			if (entry.path().extension() == ".ctrmodel")
			{
				ImportModel(entry.path());
			}
		}
	}

	//Load instances from .lev
	std::unordered_set<uint32_t> uniqueModelOffsets;
	std::vector<PSX::Vec3> instPSXPos;
	std::vector<uint32_t> instPtrs;
	if (header.numInstances > 0 && header.offModelInstances != 0)
	{
		instPSXPos.reserve(header.numInstances);
		file.seekg(offLev + std::streampos(header.offModelInstances));
		instPtrs.resize(header.numInstances);
		for (uint32_t i = 0; i < header.numInstances; i++)
		{
			Read(file, instPtrs[i]);
		}

		for (uint32_t i = 0; i < header.numInstances; i++)
		{
			if (instPtrs[i] == 0) { break; }

			file.seekg(offLev + std::streampos(instPtrs[i]));
			PSX::InstDef psxInst = {};
			Read(file, psxInst);

			Instance inst("");
			inst.SetName(std::string(psxInst.name, strnlen(psxInst.name, sizeof(psxInst.name))));
			inst.SetScale(ConvertPSXVec3(psxInst.scale, FP_ONE));
			inst.SetPos(ConvertPSXVec3(psxInst.pos, FP_ONE_GEO));
			inst.SetRot(ConvertPSXAngle(psxInst.rot));
			inst.SetModelID(static_cast<ModelId>(psxInst.modelID));
			inst.SetColor(ConvertColor(psxInst.colorRGBA));
			inst.SetFlags(psxInst.flags);
			inst.SetUnk24(psxInst.unk24);
			inst.SetUnk28(psxInst.unk28);

			// Look up model name from model data
			if (psxInst.offModel != 0)
			{
				uniqueModelOffsets.insert(psxInst.offModel);
				file.seekg(offLev + std::streampos(psxInst.offModel));
				PSX::Model psxModel = {};
				Read(file, psxModel);
				std::string modelName(psxModel.name, strnlen(psxModel.name, sizeof(psxModel.name)));
				if (!modelName.empty())
				{
					inst.SetModelName(modelName);
				}
				else
				{
					// Use synthetic name based on offset
					inst.SetModelName("LEV_Model_0x" + std::to_string(psxInst.offModel));
				}
			}

			instPSXPos.push_back(psxInst.pos);
			m_instances.push_back(inst);
		}
	}

	// Load instance hitbox data from BSP leaf offHitbox lists
	if (!m_instances.empty())
	{
		std::vector<const BSP*> bspLeaves = m_bsp.GetLeaves();
		for (const BSP* leaf : bspLeaves)
		{
			uint32_t offHitbox = leaf->GetOffHitbox();
			if (offHitbox == 0) continue;

			const uint8_t* hitboxData = m_levData.data() + 4 + offHitbox;
			for (size_t hi = 0; ; hi++)
			{
				PSX::InstHitbox hitbox = {};
				std::memcpy(&hitbox, hitboxData + hi * sizeof(PSX::InstHitbox), sizeof(PSX::InstHitbox));
				if (hitbox.flags == 0 && hitbox.offInstDef == 0)
					break;
				for (size_t i = 0; i < m_instances.size(); i++)
				{
					if (i >= instPSXPos.size()) continue;
					if (instPtrs[i] == hitbox.offInstDef)
					{
						InstanceHitbox ihb;
						ihb.enabled = true;
						ihb.flags = hitbox.flags;
						ihb.halfExtent = hitbox.halfExtent;
						ihb.yOffset = hitbox.center.y - instPSXPos[i].y;
						m_instances[i].SetHitbox(ihb);
						break;
					}
				}
			}
		}
	}

	// Auto-extract models from LEV data into .ctrmodel files and import them
	if (!uniqueModelOffsets.empty() && !m_levData.empty())
	{
		std::filesystem::path modelCacheDir = levFile.parent_path() / "extracted_models";
		std::filesystem::create_directories(modelCacheDir);

		// Extract all models using existing LevDataExtractor
		{
			std::filesystem::path extVrmPath = levFile;
			extVrmPath.replace_extension(".vrm");
			LevDataExtractor extractor(levFile, extVrmPath);
			extractor.ExtractModels();
		}

		for (uint32_t modelOff : uniqueModelOffsets)
		{
			if (modelOff == 0) continue;

			// Read model name from LEV data
			PSX::Model psxModel;
			memcpy(&psxModel, m_levData.data() + 4 + modelOff, sizeof(PSX::Model));
			std::string modelName(psxModel.name, strnlen(psxModel.name, sizeof(psxModel.name)));
			if (modelName.empty())
				modelName = "LEV_Model_0x" + std::to_string(modelOff);

			// Skip if already imported
			if (m_importedModels.find(modelName) != m_importedModels.end())
				continue;

			std::filesystem::path ctrmodelPath = modelCacheDir / (modelName + ".ctrmodel");
			if (!std::filesystem::exists(ctrmodelPath))
				continue;

			// Import the .ctrmodel (reads file into m_importedModels)
			ImportModel(ctrmodelPath);
		}
	}


	m_loaded = true;
	file.close();
	GenerateRenderLevData();
	GenerateRenderInstanceData();
	return true;
}

bool Level::SaveLEV(const std::filesystem::path& path, bool useRawTextures)
{
	/*
	*	Serialization order:
	*		- offMap
	*		- LevHeader
	*		- MeshInfo
	*		- Textures
	*		- Animated Textures
	*		- Array of quadblocks
	*		- Array of VisibleSets
	*		- Array of PVS
	*		- Array of vertices
	*		- Array of BSP
	*		- Array of checkpoints
	*		- N. Tropy Ghost
	*		- N. Oxide Ghost
	*		- LevelExtraHeader
	*		- NavHeaders
	*		- VisMem
	*		- Skybox
	*		- PointerMap
	*/
	m_hotReloadLevPath = path / (m_name + ".lev");
	std::ofstream file(m_hotReloadLevPath, std::ios::binary);

	if (m_bsp.IsEmpty()) { GenerateBSP(); }

	std::vector<const BSP*> bspNodes = m_bsp.GetTree();
	std::vector<const BSP*> orderedBSPNodes(bspNodes.size());
	for (const BSP* bsp : bspNodes) { orderedBSPNodes[bsp->GetId()] = bsp; }

	PSX::LevHeader header = {};
	const size_t offHeader = 0;
	printf(nameof(offHeader) " = %zx\n", offHeader);
	size_t currOffset = sizeof(header);

	PSX::MeshInfo meshInfo = {};
	const size_t offMeshInfo = currOffset;
	printf(nameof(offMeshInfo) " = %zx\n", offMeshInfo);
	currOffset += sizeof(meshInfo);

	const size_t offTexture = currOffset;
	printf(nameof(offTexture) " = %zx\n", offTexture);
	size_t offAnimData = 0;

	PSX::TextureLayout defaultTex = {};
	defaultTex.clut.self = 32 | (20 << 6);
	defaultTex.texPage.self = (512 >> 6) | ((0 >> 8) << 4) | (0 << 5) | (0 << 7);
	defaultTex.u0 = 0;		defaultTex.v0 = 0;
	defaultTex.u1 = 15;		defaultTex.v1 = 0;
	defaultTex.u2 = 0;		defaultTex.v2 = 15;
	defaultTex.u3 = 15;		defaultTex.v3 = 15;

	PSX::TextureGroup defaultTexGroup = {};
	defaultTexGroup.far = defaultTex;
	defaultTexGroup.middle = defaultTex;
	defaultTexGroup.near = defaultTex;
	defaultTexGroup.mosaic = defaultTex;

	std::vector<uint8_t> animData;
	std::vector<size_t> animPtrMapOffsets;
	std::vector<PSX::TextureGroup> texGroups;
	std::vector<PSX::AnimTex> animTexGroups;
	std::unordered_map<PSX::TextureLayout, size_t> savedLayouts;
	if (useRawTextures)
	{
		std::map<uint32_t, size_t> rawOffsetRemap;
		std::map<uint32_t, size_t> rawAnimOffsetRemap;
		std::vector<std::pair<size_t, size_t>> animatedQuadFaceOffsets; // <quadIndex, faceIndex> -> animTexOffset
		std::map<std::pair<size_t, size_t>, size_t> quadFaceToAnimOffset;

		for (size_t qi = 0; qi < m_quadblocks.size(); qi++)
		{
			Quadblock& currQuad = m_quadblocks[qi];
			if (currQuad.GetAnimated())
			{
				for (size_t i = 0; i < NUM_FACES_QUADBLOCK + 1; i++)
				{
					uint32_t rawTexOffset = currQuad.GetRawTexOffset(i);
					uint32_t animTexKey = rawTexOffset - 1;

					if (!m_rawAnimTex.contains(animTexKey))
					{
						// This face is not animated, treat it as a static texture
						if (!rawOffsetRemap.contains(rawTexOffset))
						{
							rawOffsetRemap[rawTexOffset] = texGroups.size();
							if (!m_rawTextureGroup.contains(rawTexOffset))
							{
								printf("MISSING TEXTURE FOR %s FACE %d\n", currQuad.GetName().c_str(), i);
							}
							texGroups.push_back(m_rawTextureGroup[rawTexOffset]);
						}
						currQuad.SetTextureID(rawOffsetRemap[rawTexOffset], i);
						continue;
					}

					const PSX::AnimTex& animTex = m_rawAnimTex[animTexKey];
					const std::vector<uint32_t>& frameOffsets = m_rawAnimTexFrames[animTexKey];

					std::vector<size_t> remappedFrameIndexes;
					for (uint32_t frameRawOffset : frameOffsets)
					{
						if (!rawOffsetRemap.contains(frameRawOffset))
						{
							rawOffsetRemap[frameRawOffset] = texGroups.size();
							if (!m_rawTextureGroup.contains(frameRawOffset))
							{
								printf("MISSING FRAME TEXTURE FOR %s FACE %d FRAME OFFSET %u\n",
									currQuad.GetName().c_str(), i, frameRawOffset);
							}
							texGroups.push_back(m_rawTextureGroup[frameRawOffset]);
						}
						remappedFrameIndexes.push_back(rawOffsetRemap[frameRawOffset]);
					}

					if (i == NUM_FACES_QUADBLOCK)
					{
						currQuad.SetTextureID(remappedFrameIndexes[0], i);
						continue;
					}

					if (!rawAnimOffsetRemap.contains(animTexKey))
					{
						size_t animTexOffset = animData.size();
						rawAnimOffsetRemap[animTexKey] = animTexOffset;
						animPtrMapOffsets.push_back(animTexOffset);

						PSX::AnimTex rawAnimTex = animTex;
						rawAnimTex.offActiveFrame = static_cast<uint32_t>(
							offTexture + (remappedFrameIndexes[0] * sizeof(PSX::TextureGroup)));
						animData.resize(animData.size() + sizeof(PSX::AnimTex));
						memcpy(&animData[animTexOffset], &rawAnimTex, sizeof(PSX::AnimTex));

						for (size_t j = 0; j < remappedFrameIndexes.size(); j++)
						{
							uint32_t offset = static_cast<uint32_t>(
								(remappedFrameIndexes[j] * sizeof(PSX::TextureGroup)) + offTexture);
							size_t offAnimTexArr = animData.size();
							animPtrMapOffsets.push_back(offAnimTexArr);
							for (size_t k = 0; k < sizeof(uint32_t); k++) { animData.push_back(0); }
							memcpy(&animData[offAnimTexArr], &offset, sizeof(uint32_t));
						}
					}

					quadFaceToAnimOffset[{qi, i}] = rawAnimOffsetRemap[animTexKey];
				}
				continue;
			}
			for (size_t i = 0; i < NUM_FACES_QUADBLOCK + 1; i++)
			{
				uint32_t rawTexOffset = currQuad.GetRawTexOffset(i);
				if (!rawOffsetRemap.contains(rawTexOffset))
				{
					rawOffsetRemap[rawTexOffset] = texGroups.size();
					if (!m_rawTextureGroup.contains(rawTexOffset)) { printf("MISSING TEXTURE FOR %s FACE %d\n", currQuad.GetName().c_str(), i); }
					texGroups.push_back(m_rawTextureGroup[rawTexOffset]);
				}
				currQuad.SetTextureID(rawOffsetRemap[rawTexOffset], i);
			}
		}

		//texGroups.push_back(defaultTexGroup);
		offAnimData = currOffset + (sizeof(PSX::TextureGroup) * texGroups.size());

		// Second pass: now offAnimData is known
		for (auto& [quadFace, animTexOffset] : quadFaceToAnimOffset)
		{
			m_quadblocks[quadFace.first].SetAnimTextureOffset(animTexOffset, offAnimData, quadFace.second);
		}

		animPtrMapOffsets.push_back(animData.size());
		size_t offEndAnimData = animData.size();
		for (size_t i = 0; i < sizeof(uint32_t); i++) { animData.push_back(0); }
		memcpy(&animData[offEndAnimData], &offAnimData, sizeof(uint32_t));
	}
	else
	{
		if (UpdateVRM())
		{
			for (auto& [material, texture] : m_materialToTexture)
			{
				std::vector<size_t>& quadIndexes = m_materialToQuadblocks[material];
				for (size_t index : quadIndexes)
				{
					Quadblock& currQuad = m_quadblocks[index];
					if (currQuad.GetAnimated()) { continue; }
					for (size_t i = 0; i < NUM_FACES_QUADBLOCK + 1; i++)
					{
						size_t textureID = 0;
						const QuadUV& uvs = currQuad.GetQuadUV(i);
						PSX::TextureLayout layout = texture.Serialize(uvs);
						if (savedLayouts.contains(layout)) { textureID = savedLayouts[layout]; }
						else
						{
							textureID = texGroups.size();
							savedLayouts[layout] = textureID;

							PSX::TextureGroup texGroup = {};
							texGroup.far = layout;
							texGroup.middle = layout;
							texGroup.near = layout;
							texGroup.mosaic = layout;
							texGroups.push_back(texGroup);
						}
						currQuad.SetTextureID(textureID, i);
					}
				}
			}

			if (!m_animTextures.empty())
			{
				std::vector<std::array<size_t, NUM_FACES_QUADBLOCK>> animOffsetPerQuadblock;
				for (AnimTexture& animTex : m_animTextures)
				{
					const std::vector<AnimTextureFrame>& animFrames = animTex.GetFrames();
					const std::vector<Texture>& animTextures = animTex.GetTextures();
					std::vector<std::vector<size_t>> texgroupIndexesPerFrame(NUM_FACES_QUADBLOCK);
					bool firstFrame = true;
					for (const AnimTextureFrame& frame : animFrames)
					{
						Texture& texture = const_cast<Texture&>(animTextures[frame.textureIndex]);
						for (size_t i = 0; i < NUM_FACES_QUADBLOCK + 1; i++)
						{
							if (i == NUM_FACES_QUADBLOCK && !firstFrame) { continue; }
							size_t textureID = 0;
							const QuadUV& uvs = frame.uvs[i];
							PSX::TextureLayout layout = texture.Serialize(uvs);
							if (savedLayouts.contains(layout)) { textureID = savedLayouts[layout]; }
							else
							{
								textureID = texGroups.size();
								savedLayouts[layout] = textureID;

								PSX::TextureGroup texGroup = {};
								texGroup.far = layout;
								texGroup.middle = layout;
								texGroup.near = layout;
								texGroup.mosaic = layout;
								texGroups.push_back(texGroup);
							}
							if (firstFrame && i == NUM_FACES_QUADBLOCK)
							{
								const std::vector<size_t>& quadblockIndexes = animTex.GetQuadblockIndexes();
								for (size_t index : quadblockIndexes)
								{
									m_quadblocks[index].SetTextureID(textureID, i);
								}
							}
							else { texgroupIndexesPerFrame[i].push_back(textureID); }
						}
						firstFrame = false;
					}
					std::array<size_t, NUM_FACES_QUADBLOCK> offsetPerQuadblock = {};
					for (size_t i = 0; i < NUM_FACES_QUADBLOCK; i++)
					{
						bool foundEquivalent = false;
						for (size_t j = 0; j < i; j++)
						{
							if (texgroupIndexesPerFrame[i] == texgroupIndexesPerFrame[j])
							{
								offsetPerQuadblock[i] = offsetPerQuadblock[j];
								foundEquivalent = true;
								break;
							}
						}
						if (foundEquivalent) { continue; }
						std::vector<uint8_t> buffer = animTex.Serialize(texgroupIndexesPerFrame[i][0], offTexture);
						size_t animTexOffset = animData.size();
						offsetPerQuadblock[i] = animTexOffset;
						animPtrMapOffsets.push_back(animTexOffset);
						for (uint8_t byte : buffer) { animData.push_back(byte); }
						for (size_t j = 0; j < animFrames.size(); j++)
						{
							uint32_t offset = static_cast<uint32_t>((texgroupIndexesPerFrame[i][j] * sizeof(PSX::TextureGroup)) + offTexture);
							size_t offAnimTexArr = animData.size();
							animPtrMapOffsets.push_back(offAnimTexArr);
							for (size_t k = 0; k < sizeof(uint32_t); k++) { animData.push_back(0); }
							memcpy(&animData[offAnimTexArr], &offset, sizeof(uint32_t));
						}
					}
					animOffsetPerQuadblock.push_back(offsetPerQuadblock);
				}

  			offAnimData = currOffset + (sizeof(PSX::TextureGroup) * texGroups.size());
	  		printf(nameof(offAnimData) " = %zx\n", offAnimData);

				animPtrMapOffsets.push_back(animData.size());
				size_t offEndAnimData = animData.size();
				for (size_t i = 0; i < sizeof(uint32_t); i++) { animData.push_back(0); }
				memcpy(&animData[offEndAnimData], &offAnimData, sizeof(uint32_t));

				for (size_t i = 0; i < m_animTextures.size(); i++)
				{
					const std::vector<size_t>& quadblockIndexes = m_animTextures[i].GetQuadblockIndexes();
					for (size_t index : quadblockIndexes)
					{
						Quadblock& quadblock = m_quadblocks[index];
						for (size_t j = 0; j < NUM_FACES_QUADBLOCK; j++)
						{
							quadblock.SetAnimTextureOffset(animOffsetPerQuadblock[i][j], offAnimData, j);
						}
					}
				}
			}
			else
			{
				offAnimData = currOffset + (sizeof(PSX::TextureGroup) * texGroups.size());
				for (size_t i = 0; i < sizeof(uint32_t); i++) { animData.push_back(0); }
				memcpy(&animData[0], &offAnimData, sizeof(uint32_t));
				animPtrMapOffsets.push_back(0);
			}

			m_hotReloadVRMPath = path / (m_name + ".vrm");
			std::ofstream vrmFile(m_hotReloadVRMPath, std::ios::binary);
			Write(vrmFile, m_vrm.data(), m_vrm.size());
			vrmFile.close();
		}
		else
		{
			texGroups.push_back(defaultTexGroup);
			offAnimData = currOffset + (sizeof(PSX::TextureGroup) * texGroups.size());
			printf(nameof(offAnimData) " = %zx\n", offAnimData);
			for (size_t i = 0; i < sizeof(uint32_t); i++) { animData.push_back(0); }
			memcpy(&animData[0], &offAnimData, sizeof(uint32_t));
			animPtrMapOffsets.push_back(0);
		}
	}
	

	currOffset += (sizeof(PSX::TextureGroup) * texGroups.size()) + animData.size();

	const size_t offQuadblocks = currOffset;
	printf(nameof(offQuadblocks) " = %zx\n", offQuadblocks);
	std::vector<std::vector<uint8_t>> serializedBSPs;
	std::vector<std::vector<uint8_t>> serializedQuads;
	std::vector<const Quadblock*> orderedQuads;
	std::unordered_map<Vertex, size_t> vertexMap;
	std::vector<Vertex> orderedVertices;
	size_t bspSize = 0;
	for (const BSP* bsp : orderedBSPNodes)
	{
		serializedBSPs.push_back(bsp->Serialize(currOffset));
		bspSize += serializedBSPs.back().size();
		if (bsp->IsBranch()) { continue; }
		const std::vector<size_t>& quadIndexes = bsp->GetQuadblockIndexes();
		for (const size_t index : quadIndexes)
		{
			const Quadblock& quadblock = m_quadblocks[index];
			std::vector<Vertex> quadVertices = quadblock.GetVertices();
			std::vector<size_t> verticesIndexes;
			for (const Vertex& vertex : quadVertices)
			{
				if (!vertexMap.contains(vertex))
				{
					size_t vertexIndex = orderedVertices.size();
					orderedVertices.push_back(vertex);
					vertexMap[vertex] = vertexIndex;
				}
				verticesIndexes.push_back(vertexMap[vertex]);
			}
			size_t quadIndex = serializedQuads.size();
			serializedQuads.push_back(quadblock.Serialize(quadIndex, offTexture, verticesIndexes));
			orderedQuads.push_back(&quadblock);
			currOffset += serializedQuads.back().size();
		}
	}

	constexpr size_t BITS_PER_SLOT = sizeof(uint32_t) * 8;
	std::vector<std::tuple<std::vector<uint32_t>, size_t>> visibleNodes;
	std::vector<std::vector<uint32_t>> uniqueVisNodes;
	std::map<std::vector<uint32_t>, size_t> visNodesOffsetMap;
	std::vector<std::tuple<std::vector<uint32_t>, size_t>> visibleQuads;
	std::vector<std::tuple<std::vector<uint32_t>, size_t>> visibleInstances;
	size_t visNodeSize = static_cast<size_t>(std::ceil(static_cast<float>(bspNodes.size()) / static_cast<float>(BITS_PER_SLOT)));
	size_t visQuadSize = static_cast<size_t>(std::ceil(static_cast<float>(m_quadblocks.size()) / static_cast<float>(BITS_PER_SLOT)));
	std::vector<uint32_t> visibleNodeAll(visNodeSize, 0xFFFFFFFF);
	for (const BSP* bsp : orderedBSPNodes)
	{
		if (bsp->GetFlags() & BSPFlags::INVISIBLE) { visibleNodeAll[bsp->GetId() / BITS_PER_SLOT] &= ~(1 << (bsp->GetId() % BITS_PER_SLOT)); }
	}

	std::vector<uint32_t> visibleQuadsAll(visQuadSize, 0xFFFFFFFF);
	size_t quadIndex = 0;
	const bool validVisTree = !m_bspVis.IsEmpty();
	const std::vector<const BSP*> bspLeaves = m_bsp.GetLeaves();
	std::unordered_map<size_t, const BSP*> idToLeaf;
	std::unordered_map<const BSP*, size_t> leafToMatrix;
	for (const BSP* leaf : bspLeaves) { idToLeaf[leaf->GetId()] = leaf; }
	for (size_t i = 0; i < bspLeaves.size(); i++) { leafToMatrix[bspLeaves[i]] = i; }
	for (const Quadblock* quad : orderedQuads)
	{
		if (quad->GetFlags() & QuadFlags::INVISIBLE_TRIGGER)
		{
			visibleQuadsAll[quadIndex / BITS_PER_SLOT] &= ~(1 << (quadIndex % BITS_PER_SLOT));
		}
		if (validVisTree)
		{
			std::vector<uint32_t> visNodes(visNodeSize, 0x0);
			const BSP* bspLeaf = idToLeaf[quad->GetBSPID()];
			const size_t matrixId = leafToMatrix[bspLeaf];
			for (size_t i = 0; i < bspLeaves.size(); i++)
			{
				if (m_bspVis.Get(matrixId, i))
				{
					const BSP* curr = bspLeaves[i];
					while (curr != nullptr)
					{
						visNodes[curr->GetId() / BITS_PER_SLOT] |= (1 << (31 - (curr->GetId() % BITS_PER_SLOT)));
						curr = curr->GetParent();
					}
				}
			}
			if (visNodesOffsetMap.contains(visNodes))
			{
				visibleNodes.push_back({ visNodes, visNodesOffsetMap.at(visNodes) });
			}
			else
			{
				visNodesOffsetMap[visNodes] = currOffset;
				visibleNodes.push_back({ visNodes, currOffset });
				uniqueVisNodes.push_back(visNodes);
				currOffset += visNodes.size() * sizeof(uint32_t);
			}
		}
		quadIndex++;
	}
	printf("visibleNodesOffsetMapSize %d\n", visNodesOffsetMap.size());
	if (!validVisTree)
	{
		visibleNodes.push_back({visibleNodeAll, currOffset});
		uniqueVisNodes.push_back(visibleNodeAll);
		currOffset += visibleNodeAll.size() * sizeof(uint32_t);
	}

	visibleQuads.push_back({visibleQuadsAll, currOffset});
	currOffset += visibleQuadsAll.size() * sizeof(uint32_t);

	std::vector<uint32_t> visibleInstancesDummy;
	visibleInstancesDummy.push_back(0xFFFFFFFF);
	visibleInstances.push_back({visibleInstancesDummy, currOffset});
	currOffset += visibleInstancesDummy.size() * sizeof(uint32_t);

	std::unordered_map<PSX::VisibleSet, size_t> visibleSetMap;
	std::vector<PSX::VisibleSet> visibleSets;
	const size_t offVisibleSet = currOffset;
	printf(nameof(offVisibleSet) " = %zx\n", offVisibleSet);

	for (size_t quadCount = 0; quadCount < orderedQuads.size(); quadCount++)
	{
		PSX::VisibleSet set = {};
		if (validVisTree) { set.offVisibleBSPNodes = static_cast<uint32_t>(std::get<size_t>(visibleNodes[quadCount])); }
		else { set.offVisibleBSPNodes = static_cast<uint32_t>(std::get<size_t>(visibleNodes[0])); }
		set.offVisibleQuadblocks = static_cast<uint32_t>(std::get<size_t>(visibleQuads[0]));
		if (orderedQuads.size() % 2 == 0 && quadCount == 0)
			set.offVisibleInstances = static_cast<uint32_t>(0);
		else
			set.offVisibleInstances = static_cast<uint32_t>(1);
		set.offVisibleExtra = 0;

		size_t visibleSetIndex = 0;
		if (visibleSetMap.contains(set)) { visibleSetIndex = visibleSetMap.at(set); }
		else
		{
			visibleSetIndex = visibleSets.size();
			visibleSets.push_back(set);
			visibleSetMap[set] = visibleSetIndex;
		}

		PSX::Quadblock* serializedQuad = reinterpret_cast<PSX::Quadblock*>(serializedQuads[quadCount].data());
		serializedQuad->offVisibleSet = static_cast<uint32_t>(offVisibleSet + sizeof(PSX::VisibleSet) * visibleSetIndex);
	}
	printf("visibleSetsSize %d\n", visibleSets.size());
	currOffset += visibleSets.size() * sizeof(PSX::VisibleSet);

	const size_t offVertices = currOffset;
	printf(nameof(offVertices) " = %zx\n", offVertices);
	std::vector<std::vector<uint8_t>> serializedVertices;
	for (const Vertex& vertex : orderedVertices)
	{
		serializedVertices.push_back(vertex.Serialize());
		currOffset += serializedVertices.back().size();
	}

	const size_t offBSP = currOffset;
	printf(nameof(offBSP) " = %zx\n", offBSP);
	currOffset += bspSize;

	meshInfo.numQuadblocks = static_cast<uint32_t>(serializedQuads.size());
	meshInfo.numVertices = static_cast<uint32_t>(serializedVertices.size());
	meshInfo.offQuadblocks = static_cast<uint32_t>(offQuadblocks);
	meshInfo.offVertices = static_cast<uint32_t>(offVertices);
	meshInfo.unk1 = 0;
	meshInfo.unk2 = 0;
	meshInfo.offBSPNodes = static_cast<uint32_t>(offBSP);
	meshInfo.numBSPNodes = static_cast<uint32_t>(serializedBSPs.size());

	const size_t offCheckpoints = currOffset;
	printf(nameof(offCheckpoints) " = %zx\n", offCheckpoints);
	std::vector<std::vector<uint8_t>> serializedCheckpoints;
	for (const Checkpoint& checkpoint : m_checkpoints)
	{
		serializedCheckpoints.push_back(checkpoint.Serialize());
		currOffset += serializedCheckpoints.back().size();
	}

	const size_t offTropyGhost = m_tropyGhost.empty() ? 0 : currOffset;
	printf(nameof(offTropyGhost) " = %zx\n", offTropyGhost);
	currOffset += m_tropyGhost.size();

	const size_t offOxideGhost = m_oxideGhost.empty() ? 0 : currOffset;
	printf(nameof(offOxideGhost) " = %zx\n", offOxideGhost);
	currOffset += m_oxideGhost.size();

	PSX::LevelExtraHeader extraHeader = {};
	if (offTropyGhost > 0)
	{
		if (offOxideGhost > 0) { extraHeader.count = PSX::LevelExtra::COUNT; }
		else { extraHeader.count = PSX::LevelExtra::N_OXIDE_GHOST; }
	}
	else { extraHeader.count = 0; }
	extraHeader.offsets[PSX::LevelExtra::MINIMAP] = 0;
	extraHeader.offsets[PSX::LevelExtra::SPAWN] = 0;
	extraHeader.offsets[PSX::LevelExtra::CAMERA_END_OF_RACE] = 0;
	extraHeader.offsets[PSX::LevelExtra::CAMERA_DEMO] = 0;
	extraHeader.offsets[PSX::LevelExtra::N_TROPY_GHOST] = static_cast<uint32_t>(offTropyGhost);
	extraHeader.offsets[PSX::LevelExtra::N_OXIDE_GHOST] = static_cast<uint32_t>(offOxideGhost);
	extraHeader.offsets[PSX::LevelExtra::CREDITS] = 0;

	const size_t offExtraHeader = currOffset;
	printf(nameof(offExtraHeader) " = %zx\n", offExtraHeader);
	currOffset += sizeof(extraHeader);

	constexpr size_t BOT_PATH_COUNT = 3;
	PSX::levAINavTable navTable{};
	std::vector<std::vector<uint8_t>> serializedBotPaths;

	const size_t offNavTable = currOffset;
	currOffset += sizeof(navTable);

	for (int i = 0; i < BOT_PATH_COUNT; i++)
	{
		if (m_botPaths[i].IsValid())
		{
			navTable.offAIPathArray[i] = currOffset;
			serializedBotPaths.push_back(m_botPaths[i].Serialize());
			currOffset += serializedBotPaths.back().size();
		}
		else
		{
			navTable.offAIPathArray[i] = 0;
		}
	}


	std::vector<uint32_t> visMemNodesP1(visNodeSize);
	const size_t offVisMemNodesP1 = currOffset;
	printf(nameof(offVisMemNodesP1) " = %zx\n", offVisMemNodesP1);
	currOffset += visMemNodesP1.size() * sizeof(uint32_t);

	std::vector<uint32_t> visMemQuadsP1(visQuadSize);
	const size_t offVisMemQuadsP1 = currOffset;
	printf(nameof(offVisMemQuadsP1) " = %zx\n", offVisMemQuadsP1);
	currOffset += visMemQuadsP1.size() * sizeof(uint32_t);

	std::vector<uint32_t> visMemBSPP1(bspNodes.size() * 2);
	const size_t offVisMemBSPP1 = currOffset;
	printf(nameof(offVisMemBSPP1) " = %zx\n", offVisMemBSPP1);
	currOffset += visMemBSPP1.size() * sizeof(uint32_t);

	PSX::VisualMem visMem = {};
	visMem.offNodes[0] = static_cast<uint32_t>(offVisMemNodesP1);
	visMem.offQuads[0] = static_cast<uint32_t>(offVisMemQuadsP1);
	visMem.offBSP[0] = static_cast<uint32_t>(offVisMemBSPP1);
	const size_t offVisMem = currOffset;
  printf(nameof(offVisMem) " = %zx\n", offVisMem);
	currOffset += sizeof(visMem);

	size_t offSkyboxData = 0;
	std::vector<uint8_t> skyboxData;
	std::vector<size_t> skyboxPtrMapOffsets;

	if (m_skybox.IsReady())
	{
		offSkyboxData = currOffset;
		printf(nameof(offSkyboxData) " = %zx\n", offSkyboxData);
		skyboxData = m_skybox.Serialize(offSkyboxData, skyboxPtrMapOffsets);
		currOffset += skyboxData.size();
	}

	header.offMeshInfo = static_cast<uint32_t>(offMeshInfo);
	header.offAnimTex = static_cast<uint32_t>(offAnimData);
	for (size_t i = 0; i < NUM_DRIVERS; i++)
	{
		header.driverSpawn[i].pos = ConvertVec3(m_spawn[i].pos, FP_ONE_GEO);
		header.driverSpawn[i].rot = ConvertAngle(m_spawn[i].rot);
	}
	header.config = m_configFlags;
	for (size_t i = 0; i < NUM_GRADIENT; i++)
	{
		header.skyGradient[i].posFrom = ConvertFloat(m_skyGradient[i].posFrom, 1u);
		header.skyGradient[i].posTo = ConvertFloat(m_skyGradient[i].posTo, 1u);
		header.skyGradient[i].colorFrom = ConvertColor(m_skyGradient[i].colorFrom);
		header.skyGradient[i].colorTo = ConvertColor(m_skyGradient[i].colorTo);
	}
	header.stars = ConvertStars(m_stars);
	header.jumpYSpeedCap = static_cast<uint32_t>(m_jumpYSpeedCap);
	header.splitLines[0] = ConvertFloat(m_splitLines[0], FP_ONE_GEO);
	header.splitLines[1] = ConvertFloat(m_splitLines[1], FP_ONE_GEO);
	header.offExtra = static_cast<uint32_t>(offExtraHeader);
	header.numCheckpointNodes = static_cast<uint32_t>(m_checkpoints.size());
	header.offCheckpointNodes = static_cast<uint32_t>(offCheckpoints);
	header.offVisMem = static_cast<uint32_t>(offVisMem);
	header.offLevNavTable = static_cast<uint32_t>(offNavTable);
	
	// Set skybox pointer in header if enabled
	if (m_skybox.IsReady())
	{
		header.offSkybox = static_cast<uint32_t>(offSkyboxData);
	}

	// Count unique models referenced by instances
	std::unordered_set<std::string> uniqueModelNames;
	for (const Instance& inst : m_instances)
	{
		uniqueModelNames.insert(inst.GetModelName());
	}

	header.numInstances = static_cast<uint32_t>(m_instances.size());
	header.numModels = static_cast<uint32_t>(uniqueModelNames.size());

	// Write InstDefs
	size_t offInstDefArray = currOffset;
	std::vector<size_t> instDefOffsets;
	std::vector<std::vector<uint8_t>> serializedInstDef;
	for (size_t i = 0; i < m_instances.size(); i++)
	{
		serializedInstDef.push_back(m_instances[i].Serialize());

		const size_t offInstDef = currOffset;
		instDefOffsets.push_back(offInstDef);
		printf("offInstDef[%zu] = %zx\n", i, offInstDef);
		currOffset += sizeof(PSX::InstDef);
	}

	// Write InstDef pointer array (NULL-terminated)
	const size_t offInstDefList_ptrArray = currOffset;
	printf(nameof(offInstDefList_ptrArray) " = %zx\n", offInstDefList_ptrArray);
	currOffset += (instDefOffsets.size() + 1) * sizeof(uint32_t);

	// Write second InstDef pointer array for visibility (NULL-terminated)
	const size_t offInstDefList2_ptrArray = currOffset;
	printf(nameof(offInstDefList2_ptrArray) " = %zx\n", offInstDefList2_ptrArray);
	currOffset += (instDefOffsets.size() + 1) * sizeof(uint32_t);

	// Write third InstDef pointer array for visibility even quadcount (NULL-terminated)
	const size_t offInstDefList3_ptrArray = currOffset;
	printf(nameof(offInstDefList3_ptrArray) " = %zx\n", offInstDefList3_ptrArray);
	currOffset += (instDefOffsets.size() + 1) * sizeof(uint32_t);

	// Update visible sets to point to InstDef list
	bool first = true;
	for (auto& set : visibleSets)
	{
		size_t index = visibleSetMap[set];
		visibleSetMap.erase(set);
		if (first && orderedQuads.size() % 2 == 0)
			set.offVisibleInstances = offInstDefList3_ptrArray;
		else 
			set.offVisibleInstances = offInstDefList2_ptrArray;
		visibleSetMap[set] = index;
		first = false;
	}

	header.offInstances = (m_instances.size() > 0) ? static_cast<uint32_t>(offInstDefArray) : 0;
	header.offModelInstances = static_cast<uint32_t>(offInstDefList_ptrArray);

	// Write Model data for each unique model
	std::unordered_map<std::string, size_t> modelOffsets;
	std::vector<std::string> modelOrder(uniqueModelNames.begin(), uniqueModelNames.end());

	for (const std::string& modelName : modelOrder)
	{
		const std::vector<uint8_t>& ctrmodelData = m_importedModels.at(modelName);

		// Parse .ctrmodel to get model data size
		const SH::CtrModel* ctrHeader = reinterpret_cast<const SH::CtrModel*>(ctrmodelData.data());
		size_t modelDataSize = ctrHeader->modelPatchTableOffset - ctrHeader->modelOffset;

		const size_t offModel = currOffset;
		modelOffsets[modelName] = offModel;
		printf("offModel[%s] = %zx (%zu bytes)\n", modelName.c_str(), offModel, modelDataSize);
		currOffset += modelDataSize;
	}

	// Write Model pointer array (NULL-terminated)
	const size_t offModelList_ptrArray = currOffset;
	printf(nameof(offModelList_ptrArray) " = %zx\n", offModelList_ptrArray);
	currOffset += (uniqueModelNames.size() + 1) * sizeof(uint32_t);
	header.offModels = static_cast<uint32_t>(offModelList_ptrArray);


	//Update Insdef offModel
	for (size_t i = 0; i < serializedInstDef.size(); i++)
	{
		PSX::InstDef* inst = reinterpret_cast<PSX::InstDef*>(serializedInstDef[i].data());
		inst->offModel = static_cast<uint32_t>(modelOffsets[m_instances[i].GetModelName()]);
	}

	// Build BSP-leaf instance hitbox lists.
	// Each BSP leaf whose bbox overlaps an enabled hitbox gets a list of
	// InstHitbox entries (one per overlapping instance) plus a 4-byte
	// terminator. The leaf's offHitbox field (already serialized into
	// serializedBSPs) is patched to point at its list.

	// NOTE : Shouldn't be done here. Must be done within BSP creation, and serialized with BSP.
	struct LeafHitboxList
	{
		size_t leafFileOffset; // file offset of the BSP leaf node
		size_t listFileOffset; // file offset of this hitbox list
		std::vector<PSX::InstHitbox> entries;
	};
	std::vector<LeafHitboxList> leafHitboxLists;
	{
		std::vector<PSX::InstHitbox> enabledHitboxes;
		for (size_t i = 0; i < m_instances.size(); i++)
		{
			const InstanceHitbox& settings = m_instances[i].GetHitbox();
			if (!settings.enabled) { continue; }
			PSX::InstDef* inst = reinterpret_cast<PSX::InstDef*>(serializedInstDef[i].data());
			const int he = settings.halfExtent;
			const int cx = inst->pos.x;
			const int cy = inst->pos.y + settings.yOffset;
			const int cz = inst->pos.z;

			PSX::InstHitbox hitbox = {};
			hitbox.flags = settings.flags;
			hitbox.bbox.min = {static_cast<int16_t>(cx - he), static_cast<int16_t>(cy - he), static_cast<int16_t>(cz - he)};
			hitbox.bbox.max = {static_cast<int16_t>(cx + he), static_cast<int16_t>(cy + he), static_cast<int16_t>(cz + he)};
			hitbox.center = {static_cast<int16_t>(cx), static_cast<int16_t>(cy), static_cast<int16_t>(cz)};
			hitbox.halfExtent = static_cast<int16_t>(he);
			hitbox.halfExtentSq = static_cast<int16_t>(he * he);
			hitbox.padding = 0;
			hitbox.offInstDef = static_cast<uint32_t>(instDefOffsets[i]);
			enabledHitboxes.push_back(hitbox);
		}

		if (!enabledHitboxes.empty())
		{
			size_t nodeFileOffset = offBSP;
			for (size_t node = 0; node < serializedBSPs.size(); node++)
			{
				const size_t currNodeOffset = nodeFileOffset;
				nodeFileOffset += serializedBSPs[node].size();
				if (orderedBSPNodes[node]->IsBranch()) { continue; }

				PSX::BSPLeaf* leaf = reinterpret_cast<PSX::BSPLeaf*>(serializedBSPs[node].data());
				std::vector<PSX::InstHitbox> overlapping;
				for (const PSX::InstHitbox& hitbox : enabledHitboxes)
				{
					if (hitbox.bbox.max.x < leaf->bbox.min.x || leaf->bbox.max.x < hitbox.bbox.min.x ||
							hitbox.bbox.max.y < leaf->bbox.min.y || leaf->bbox.max.y < hitbox.bbox.min.y ||
							hitbox.bbox.max.z < leaf->bbox.min.z || leaf->bbox.max.z < hitbox.bbox.min.z) { continue; }
					overlapping.push_back(hitbox);
				}
				if (overlapping.empty()) { continue; }

				leaf->offHitbox = static_cast<uint32_t>(currOffset);
				printf("offLeafHitboxList[node %zu] = %zx (%zu entries)\n", node, currOffset, overlapping.size());
				leafHitboxLists.push_back({currNodeOffset, currOffset, std::move(overlapping)});
				currOffset += leafHitboxLists.back().entries.size() * sizeof(PSX::InstHitbox) + sizeof(uint32_t); // entries + terminator
			}
		}
	}

	size_t paddingSizeForMultOfFour = (4 - (currOffset % 4)) % 4;
	printf(nameof(paddingSizeForMultOfFour) " = %zx\n", paddingSizeForMultOfFour);
	currOffset += paddingSizeForMultOfFour;

	const size_t offPointerMap = currOffset;
	printf(nameof(offPointerMap) " = %zx\n", offPointerMap);

	std::vector<uint32_t> pointerMap =
	{
		CALCULATE_OFFSET(PSX::LevHeader, offMeshInfo, offHeader),
		CALCULATE_OFFSET(PSX::LevHeader, offInstances, offHeader),
		CALCULATE_OFFSET(PSX::LevHeader, offModels, offHeader),
		CALCULATE_OFFSET(PSX::LevHeader, offModelInstances, offHeader),
		CALCULATE_OFFSET(PSX::LevHeader, offExtra, offHeader),
		CALCULATE_OFFSET(PSX::LevHeader, offCheckpointNodes, offHeader),
		CALCULATE_OFFSET(PSX::LevHeader, offVisMem, offHeader),
		CALCULATE_OFFSET(PSX::LevHeader, offAnimTex, offHeader),
		CALCULATE_OFFSET(PSX::LevHeader, offLevNavTable, offHeader),
		CALCULATE_OFFSET(PSX::MeshInfo, offQuadblocks, offMeshInfo),
		CALCULATE_OFFSET(PSX::MeshInfo, offVertices, offMeshInfo),
		CALCULATE_OFFSET(PSX::MeshInfo, offBSPNodes, offMeshInfo),
		CALCULATE_OFFSET(PSX::VisualMem, offNodes[0], offVisMem),
		CALCULATE_OFFSET(PSX::VisualMem, offQuads[0], offVisMem),
		CALCULATE_OFFSET(PSX::VisualMem, offBSP[0], offVisMem),
	};

	// Add InstDef.offModel pointers
	for (size_t i = 0; i < m_instances.size(); i++)
	{
		pointerMap.push_back(CALCULATE_OFFSET(PSX::InstDef, offModel, instDefOffsets[i]));
	}

	// Add pointer array entries
	for (size_t i = 0; i < instDefOffsets.size(); i++)
	{
		pointerMap.push_back(static_cast<uint32_t>(offInstDefList_ptrArray + (i * sizeof(uint32_t))));
		pointerMap.push_back(static_cast<uint32_t>(offInstDefList2_ptrArray + (i * sizeof(uint32_t))));
		pointerMap.push_back(static_cast<uint32_t>(offInstDefList3_ptrArray + (i * sizeof(uint32_t))));
	}

	for (size_t i = 0; i < modelOrder.size(); i++)
	{
		pointerMap.push_back(static_cast<uint32_t>(offModelList_ptrArray + (i * sizeof(uint32_t))));
	}

	// Add model internal pointers to .lev patch table
	for (const std::string& modelName : modelOrder)
	{
		const std::vector<uint8_t>& ctrmodelData = m_importedModels.at(modelName);
		size_t modelBaseOffset = modelOffsets[modelName];

		// Parse .ctrmodel to get patch table
		const SH::CtrModel* ctrHeader = reinterpret_cast<const SH::CtrModel*>(ctrmodelData.data());
		const uint32_t* patchTablePtr = reinterpret_cast<const uint32_t*>(ctrmodelData.data() + ctrHeader->modelPatchTableOffset);
		const uint32_t patchCount = *patchTablePtr;
		const uint32_t* patchOffsets = patchTablePtr + 1;

		// Add each pointer field location to .lev patch table
		for (uint32_t i = 0; i < patchCount; i++)
		{
			uint32_t ctrPatchOffset = patchOffsets[i]; // Absolute offset in .ctrmodel where pointer field is
			uint32_t relativeOffset = ctrPatchOffset - ctrHeader->modelOffset; // Relative to model data
			uint32_t levPatchOffset = static_cast<uint32_t>(modelBaseOffset + relativeOffset); // Absolute in .lev
			pointerMap.push_back(levPatchOffset);
		}
	}

	// Add skybox header pointer to pointer map
	if (m_skybox.IsReady())
	{
		pointerMap.push_back(CALCULATE_OFFSET(PSX::LevHeader, offSkybox, offHeader));
	}

	// Add BSP-leaf hitbox pointers: each leaf's offHitbox field, plus the
	// InstDef pointer inside every hitbox entry
	for (const LeafHitboxList& list : leafHitboxLists)
	{
		pointerMap.push_back(CALCULATE_OFFSET(PSX::BSPLeaf, offHitbox, list.leafFileOffset));
		for (size_t i = 0; i < list.entries.size(); i++)
		{
			pointerMap.push_back(CALCULATE_OFFSET(PSX::InstHitbox, offInstDef, list.listFileOffset + (i * sizeof(PSX::InstHitbox))));
		}
	}

	if (offTropyGhost != 0) { pointerMap.push_back(CALCULATE_OFFSET(PSX::LevelExtraHeader, offsets[PSX::LevelExtra::N_TROPY_GHOST], offExtraHeader)); }
	if (offOxideGhost != 0) { pointerMap.push_back(CALCULATE_OFFSET(PSX::LevelExtraHeader, offsets[PSX::LevelExtra::N_OXIDE_GHOST], offExtraHeader)); }

	for (size_t i = 0; i < animPtrMapOffsets.size(); i++)
	{
		pointerMap.push_back(static_cast<uint32_t>(animPtrMapOffsets[i] + offAnimData));
	}

	size_t offCurrQuad = offQuadblocks;
	for (size_t i = 0; i < serializedQuads.size(); i++)
	{
		pointerMap.push_back(CALCULATE_OFFSET(PSX::Quadblock, offMidTextures[0], offCurrQuad));
		pointerMap.push_back(CALCULATE_OFFSET(PSX::Quadblock, offMidTextures[1], offCurrQuad));
		pointerMap.push_back(CALCULATE_OFFSET(PSX::Quadblock, offMidTextures[2], offCurrQuad));
		pointerMap.push_back(CALCULATE_OFFSET(PSX::Quadblock, offMidTextures[3], offCurrQuad));
		pointerMap.push_back(CALCULATE_OFFSET(PSX::Quadblock, offLowTexture, offCurrQuad));
		pointerMap.push_back(CALCULATE_OFFSET(PSX::Quadblock, offVisibleSet, offCurrQuad));
		offCurrQuad += serializedQuads[i].size();
	}

	size_t offCurrNode = offBSP;
	for (size_t i = 0; i < serializedBSPs.size(); i++)
	{
		if (orderedBSPNodes[i]->IsBranch()) { offCurrNode += serializedBSPs[i].size(); continue; }
		size_t visMemListIndex = 2 * i + 1;
		visMemBSPP1[visMemListIndex] = static_cast<uint32_t>(offCurrNode);
		visMemBSPP1[visMemListIndex - 1] = static_cast<uint32_t>(offInstDefList2_ptrArray);
		pointerMap.push_back(static_cast<uint32_t>(offVisMemBSPP1 + visMemListIndex * sizeof(uint32_t)));
		pointerMap.push_back(static_cast<uint32_t>(offVisMemBSPP1 + (visMemListIndex - 1) * sizeof(uint32_t)));
		pointerMap.push_back(CALCULATE_OFFSET(PSX::BSPLeaf, offQuads, offCurrNode));
		offCurrNode += serializedBSPs[i].size();
	}

	size_t offCurrVisibleSet = offVisibleSet;
	for (const PSX::VisibleSet& visibleSet : visibleSets)
	{
		pointerMap.push_back(CALCULATE_OFFSET(PSX::VisibleSet, offVisibleBSPNodes, offCurrVisibleSet));
		pointerMap.push_back(CALCULATE_OFFSET(PSX::VisibleSet, offVisibleQuadblocks, offCurrVisibleSet));
		pointerMap.push_back(CALCULATE_OFFSET(PSX::VisibleSet, offVisibleInstances, offCurrVisibleSet));
		offCurrVisibleSet += sizeof(PSX::VisibleSet);
	}

	// Add skybox internal pointers to pointer map
	for (size_t offset : skyboxPtrMapOffsets)
	{
		pointerMap.push_back(static_cast<uint32_t>(offset));
	}

	for (size_t i = 0; i < 3; i++)
	{
		if (m_botPaths[i].IsValid())
			pointerMap.push_back(CALCULATE_OFFSET(PSX::levAINavTable, offAIPathArray[i], offNavTable));
	}
  
  #undef CALCULATE_OFFSET

	const size_t pointerMapBytes = pointerMap.size() * sizeof(uint32_t);

	Write(file, &offPointerMap, sizeof(uint32_t));
	Write(file, &header, sizeof(header));
	Write(file, &meshInfo, sizeof(meshInfo));
	Write(file, texGroups.data(), texGroups.size() * sizeof(PSX::TextureGroup));
	if (!animData.empty()) { Write(file, animData.data(), animData.size()); }
	for (const std::vector<uint8_t>& serializedQuad : serializedQuads) { Write(file, serializedQuad.data(), serializedQuad.size()); }
	for (const auto& visNode : uniqueVisNodes)
	{
		Write(file, visNode.data(), visNode.size() * sizeof(uint32_t));
	}
	for (const auto& tuple : visibleQuads)
	{
		const std::vector<uint32_t>& visibleQuad = std::get<0>(tuple);
		Write(file, visibleQuad.data(), visibleQuad.size() * sizeof(uint32_t));
	}
	for (const auto& tuple : visibleInstances)
	{
		const std::vector<uint32_t>& visibleInst = std::get<0>(tuple);
		Write(file, visibleInst.data(), visibleInst.size() * sizeof(uint32_t));
	}
	Write(file, visibleSets.data(), visibleSets.size() * sizeof(PSX::VisibleSet));
	for (const std::vector<uint8_t>& serializedVertex : serializedVertices) { Write(file, serializedVertex.data(), serializedVertex.size()); }
	for (const std::vector<uint8_t>& serializedBSP : serializedBSPs) { Write(file, serializedBSP.data(), serializedBSP.size()); }
	for (const std::vector<uint8_t>& serializedCheckpoint : serializedCheckpoints) { Write(file, serializedCheckpoint.data(), serializedCheckpoint.size()); }
	if (!m_tropyGhost.empty()) { Write(file, m_tropyGhost.data(), m_tropyGhost.size()); }
	if (!m_oxideGhost.empty()) { Write(file, m_oxideGhost.data(), m_oxideGhost.size()); }
	Write(file, &extraHeader, sizeof(extraHeader));
	Write(file, &navTable, sizeof(navTable));
	for (const std::vector<uint8_t>& serializedBotPath : serializedBotPaths) { Write(file, serializedBotPath.data(), serializedBotPath.size()); }
	Write(file, visMemNodesP1.data(), visMemNodesP1.size() * sizeof(uint32_t));
	Write(file, visMemQuadsP1.data(), visMemQuadsP1.size() * sizeof(uint32_t));
	Write(file, visMemBSPP1.data(), visMemBSPP1.size() * sizeof(uint32_t));
	Write(file, &visMem, sizeof(visMem));

	// Write skybox data immediately after visMem, matching the offset accounting
	// above (offSkyboxData = currOffset right after sizeof(visMem)). The rebase onto
	// the skybox-enabled main left this write at the end of the file while the
	// accounting kept it here, corrupting header.offSkybox and every instance/model
	// /hitbox offset that follows.
	if (!skyboxData.empty()) { Write(file, skyboxData.data(), skyboxData.size()); }

	// Write InstDefs
	for (const std::vector<uint8_t>& inst : serializedInstDef) { Write(file, inst.data(), inst.size()); }

	// Write InstDef pointer arrays (NULL-terminated, stored offsets - game adds 4 to get actual position)
	for (size_t offset : instDefOffsets)
	{
		uint32_t ptr = static_cast<uint32_t>(offset);
		Write(file, &ptr, sizeof(ptr));
	}
	uint32_t nullTerm = 0;
	Write(file, &nullTerm, sizeof(nullTerm));

	// Write second InstDef pointer array
	for (size_t offset : instDefOffsets)
	{
		uint32_t ptr = static_cast<uint32_t>(offset);
		Write(file, &ptr, sizeof(ptr));
	}
	Write(file, &nullTerm, sizeof(nullTerm));

	// Write third InstDef pointer array
	for (size_t offset : instDefOffsets)
	{
		uint32_t ptr = static_cast<uint32_t>(offset);
		Write(file, &ptr, sizeof(ptr));
	}
	Write(file, &nullTerm, sizeof(nullTerm));

	// Write Model data with pointer conversion from .ctrmodel to .lev format
	for (const std::string& modelName : modelOrder)
	{
		const std::vector<uint8_t>& ctrmodelData = m_importedModels.at(modelName);
		size_t modelBaseOffset = modelOffsets[modelName];

		// Parse .ctrmodel to get model data and patch table
		const SH::CtrModel* ctrHeader = reinterpret_cast<const SH::CtrModel*>(ctrmodelData.data());
		const uint8_t* modelDataSrc = ctrmodelData.data() + ctrHeader->modelOffset;
		size_t modelDataSize = ctrHeader->modelPatchTableOffset - ctrHeader->modelOffset;

		const uint32_t* patchTablePtr = reinterpret_cast<const uint32_t*>(ctrmodelData.data() + ctrHeader->modelPatchTableOffset);
		const uint32_t patchCount = *patchTablePtr;
		const uint32_t* patchOffsets = patchTablePtr + 1;

		// Copy the model data
		std::vector<uint8_t> modelData(modelDataSrc, modelDataSrc + modelDataSize);

		// Patch TextureLayouts with new VRAM coordinates
		// Parse Model and ModelHeaders to find TextureLayout arrays
		const PSX::Model* model = reinterpret_cast<const PSX::Model*>(modelData.data());
		const uint32_t modelHeadersOffset = model->offHeaders - ctrHeader->modelOffset;
		const PSX::ModelHeader* modelHeaders = reinterpret_cast<const PSX::ModelHeader*>(modelData.data() + modelHeadersOffset);

		for (uint8_t h = 0; h < model->numHeaders; h++)
		{
			const PSX::ModelHeader& modelHdr = modelHeaders[h];
			if (modelHdr.offTexLayout == 0) { continue; }

			// offTexLayout points to a pointer array, each entry points to a TextureLayout
			uint32_t ptrArrayOffset = modelHdr.offTexLayout - ctrHeader->modelOffset;
			const uint32_t* texLayoutPtrs = reinterpret_cast<const uint32_t*>(modelData.data() + ptrArrayOffset);

			// Count TextureLayouts by finding the first null or out-of-range pointer
			size_t numLayouts = 0;
			while (texLayoutPtrs[numLayouts] != 0 &&
			       texLayoutPtrs[numLayouts] >= ctrHeader->modelOffset &&
			       texLayoutPtrs[numLayouts] < ctrHeader->modelPatchTableOffset)
			{
				numLayouts++;
			}

			for (size_t i = 0; i < numLayouts; i++)
			{
				uint32_t layoutOffset = texLayoutPtrs[i] - ctrHeader->modelOffset;
				PSX::TextureLayout* layout = reinterpret_cast<PSX::TextureLayout*>(modelData.data() + layoutOffset);

				// Extract original texpage/clut from layout
				uint8_t origPageX = layout->texPage.x;
				uint8_t origPageY = layout->texPage.y;
				uint8_t origPalX = layout->clut.x;
				uint16_t origPalY = layout->clut.y;

				// Find matching ModelTextureForVRM
				for (const ModelTextureForVRM& tex : m_modelTexturesInVRAM)
				{
					if (tex.modelName != modelName) { continue; }
					if (!tex.placed) { continue; }
					if (tex.origPageX != origPageX) { continue; }
					if (tex.origPageY != origPageY) { continue; }
					if (tex.origPalX != origPalX) { continue; }
					if (tex.origPalY != origPalY) { continue; }

					// Found matching texture! Update TextureLayout with new coordinates
					// Internal buffer position â VRAM position: add 512 to X (VRM is placed at VRAM X=512)
					size_t vramX = 512 + tex.imageX;
					size_t vramY = tex.imageY;

					// Calculate new texpage (64x256 pages)
					layout->texPage.x = static_cast<uint16_t>(vramX / 64);
					layout->texPage.y = static_cast<uint16_t>(vramY / 256);
					layout->texPage.blendMode = tex.blendMode;
					layout->texPage.texpageColors = tex.bpp;

					// Calculate new CLUT coords (if indexed)
					if (tex.bpp < 2)
					{
						size_t clutVramX = 512 + tex.clutX;
						size_t clutVramY = tex.clutY;
						layout->clut.x = static_cast<uint16_t>(clutVramX / 16);
						layout->clut.y = static_cast<uint16_t>(clutVramY);
					}

					// Calculate UV adjustment
					// The texture was extracted starting at UV (originU, originV)
					// Now it's placed at position (vramX % 64, vramY % 256) within the new texpage
					// UV coordinates are scaled by BPP: 4bpp=4x, 8bpp=2x, 16bpp=1x
					int uvStretch = (tex.bpp == 0) ? 4 : (tex.bpp == 1) ? 2 : 1;
					int newOriginU = static_cast<int>((vramX % 64) * uvStretch);
					int newOriginV = static_cast<int>(vramY % 256);
					int deltaU = newOriginU - tex.originU;
					int deltaV = newOriginV - tex.originV;

					printf("uvs stuff\n");

					if (deltaU != 0 || deltaV != 0)
					{
						printf("UV adjust %s tex[%zu]: originU=%d originV=%d -> newOriginU=%d newOriginV=%d (deltaU=%d deltaV=%d)\n",
						       modelName.c_str(), tex.textureIndex, tex.originU, tex.originV, newOriginU, newOriginV, deltaU, deltaV);
						printf("  vramX=%zu vramY=%zu, texpage=(%d,%d), bpp=%d, stretch=%d\n",
						       vramX, vramY, layout->texPage.x, layout->texPage.y, tex.bpp, uvStretch);
					}

					// Adjust all UV coordinates
					layout->u0 = static_cast<uint8_t>(layout->u0 + deltaU);
					layout->v0 = static_cast<uint8_t>(layout->v0 + deltaV);
					layout->u1 = static_cast<uint8_t>(layout->u1 + deltaU);
					layout->v1 = static_cast<uint8_t>(layout->v1 + deltaV);
					layout->u2 = static_cast<uint8_t>(layout->u2 + deltaU);
					layout->v2 = static_cast<uint8_t>(layout->v2 + deltaV);
					layout->u3 = static_cast<uint8_t>(layout->u3 + deltaU);
					layout->v3 = static_cast<uint8_t>(layout->v3 + deltaV);

					break; // Found and patched
				}
			}
		}

		// Convert pointers from .ctrmodel format to .lev format
		// .ctrmodel: absolute offsets pointing directly to targets
		// .lev: stored offsets where (stored + 4) = actual file position
		// Since modelBaseOffset is already a stored offset (actual - 4), we just compute:
		// new_stored_offset = modelBaseOffset + relative_offset_within_model
		for (uint32_t i = 0; i < patchCount; i++)
		{
			uint32_t ctrPatchOffset = patchOffsets[i]; // Absolute offset in .ctrmodel where pointer field is
			uint32_t relativeOffset = ctrPatchOffset - ctrHeader->modelOffset; // Relative to model data

			if (relativeOffset + sizeof(uint32_t) <= modelData.size())
			{
				uint32_t* ptrLocation = reinterpret_cast<uint32_t*>(&modelData[relativeOffset]);
				uint32_t ctrPointerValue = *ptrLocation; // Absolute in .ctrmodel, points directly to target

				// Transform to .lev stored offset format
				// Target's relative position within model = ctrPointerValue - ctrModelOffset
				// Target's stored offset in .lev = modelBaseOffset + relative_position
				uint32_t levPointerValue = static_cast<uint32_t>(
					modelBaseOffset + (ctrPointerValue - ctrHeader->modelOffset)
				);
				*ptrLocation = levPointerValue;
			}
		}

		// Write the modified model data
		Write(file, modelData.data(), modelData.size());
	}

	// Write Model pointer array (NULL-terminated, stored offsets - game adds 4 to get actual position)
	for (const std::string& modelName : modelOrder)
	{
		uint32_t ptr = static_cast<uint32_t>(modelOffsets[modelName]);
		Write(file, &ptr, sizeof(ptr));
	}
	Write(file, &nullTerm, sizeof(nullTerm));

	// Write BSP-leaf instance hitbox lists (each NULL-terminated)
	for (const LeafHitboxList& list : leafHitboxLists)
	{
		Write(file, list.entries.data(), list.entries.size() * sizeof(PSX::InstHitbox));
		Write(file, &nullTerm, sizeof(nullTerm));
	}

	uint32_t fourBytesOfZero = 0;
	if (paddingSizeForMultOfFour > 0)
	{
		printf("WARNING: HAD TO PAD %zu BYTES\n", paddingSizeForMultOfFour);
		Write(file, &fourBytesOfZero, paddingSizeForMultOfFour);
	}
	Write(file, &pointerMapBytes, sizeof(uint32_t));
	Write(file, pointerMap.data(), pointerMapBytes);
	file.close();
	return true;
}

bool Level::LoadOBJ(const std::filesystem::path& objFile, bool isLevel)
{
	std::string line;
	std::ifstream file(objFile);
	m_name = objFile.filename().replace_extension().string();
	m_parentPath = objFile.parent_path();

	m_hasRawTexture = false;

	bool ret = true;
	std::unordered_map<std::string, std::vector<Tri>> triMap;
	std::unordered_map<std::string, std::vector<Quad>> quadMap;
	std::unordered_map<std::string, std::vector<Vec3>> normalMap;
	std::unordered_map<std::string, std::string> materialMap;
	std::unordered_map<std::string, bool> meshMap;
	std::unordered_set<std::string> materials;
	std::vector<Point> vertices;
	std::vector<Vec3> normals;
	std::vector<Vec2> uvs;
	std::string currQuadblockName;
	bool currQuadblockGoodUV = true;
	size_t quadblockCount = 0;
	while (std::getline(file, line))
	{
		std::vector<std::string> tokens = Split(line);
		if (tokens.empty()) { continue; }
		const std::string& command = tokens[0];
		if (command == "v")
		{
			if (tokens.size() < 4) { continue; }
			vertices.emplace_back(std::stof(tokens[1]), std::stof(tokens[2]), std::stof(tokens[3]));
			if (tokens.size() < 7) { continue; }
			vertices.back().color = Color(std::stof(tokens[4]), std::stof(tokens[5]), std::stof(tokens[6]));
		}
		else if (command == "vn")
		{
			if (tokens.size() < 4) { continue; }
			normals.emplace_back(std::stof(tokens[1]), std::stof(tokens[2]), std::stof(tokens[3]));
		}
		else if (command == "vt")
		{
			if (tokens.size() < 3) { continue; }
			Vec2 uv = {std::stof(tokens[1]), std::stof(tokens[2])};
			if (uv.x < 0.0f || uv.x > 1.0f || uv.y < 0.0f || uv.y > 1.0f)
			{
				m_invalidQuadblocks.emplace_back(currQuadblockName, "WARNING: UV outside of expect range [0.0f, 1.0f].");
			}
			auto Wrap = [](float x)
				{
					if (x >= 0.0f && x <= 1.0f) { return x; }
					float r = fmodf(x, 1.0f);
					if (r < 0.0f) { r += 1.0f; }
					if (r == 0.0f && x > 0.0f) { r = 1.0f; }
					return r;
				};
			uv.x = Wrap(uv.x);
			uv.y = 1.0f - Wrap(uv.y);
			uvs.emplace_back(uv);
		}
		else if (command == "o")
		{
			if (tokens.size() < 2 || meshMap.contains(tokens[1]))
			{
				ret = false;
				m_invalidQuadblocks.emplace_back(tokens[1], "Duplicated mesh name.");
				continue;
			}
			currQuadblockName = tokens[1];
			currQuadblockGoodUV = true;
			meshMap[currQuadblockName] = false;
			quadblockCount++;
		}
		else if (command == "usemtl")
		{
			if (tokens.size() < 2) { continue; }
			if (currQuadblockName.empty() || materialMap.contains(currQuadblockName)) { continue; } /* TODO: return false, generate error message */
			materialMap[currQuadblockName] = tokens[1];
		}
		else if (command == "f")
		{
			if (currQuadblockName.empty()) { return false; }
			if (tokens.size() < 4) { continue; }

			if (meshMap.contains(currQuadblockName) && meshMap.at(currQuadblockName))
			{
				ret = false;
				m_invalidQuadblocks.emplace_back(currQuadblockName, "Triblock and Quadblock merged in the same mesh.");
				continue;
			}

			bool isQuadblock = tokens.size() == 5;

			std::vector<std::string> token0 = Split(tokens[1], '/');
			std::vector<std::string> token1 = Split(tokens[2], '/');
			std::vector<std::string> token2 = Split(tokens[3], '/');

			const size_t EXPECTED_INFORMATION_PER_TOKEN = 3; /* pos, opt uvs, normals */
			if (token0.size() < EXPECTED_INFORMATION_PER_TOKEN ||
				token1.size() < EXPECTED_INFORMATION_PER_TOKEN ||
				token2.size() < EXPECTED_INFORMATION_PER_TOKEN)
			{
				ret = false;
				m_invalidQuadblocks.emplace_back(currQuadblockName, "Missing vertex normals.");
				continue;
			}

			int i0 = std::stoi(token0[0]) - 1;
			int i1 = std::stoi(token1[0]) - 1;
			int i2 = std::stoi(token2[0]) - 1;
			int ni0 = std::stoi(token0[2]) - 1;
			int ni1 = std::stoi(token1[2]) - 1;
			int ni2 = std::stoi(token2[2]) - 1;
			normalMap[currQuadblockName].push_back(normals[ni0]);
			normalMap[currQuadblockName].push_back(normals[ni1]);
			normalMap[currQuadblockName].push_back(normals[ni2]);

			vertices[i0].normal = normals[ni0];
			vertices[i1].normal = normals[ni1];
			vertices[i2].normal = normals[ni2];

			if (currQuadblockGoodUV)
			{
				int uv0 = 0;
				int uv1 = 0;
				int uv2 = 0;
				try
				{
					uv0 = std::stoi(token0[1]) - 1;
					uv1 = std::stoi(token1[1]) - 1;
					uv2 = std::stoi(token2[1]) - 1;
				}
				catch (...) { currQuadblockGoodUV = false; }

				if (currQuadblockGoodUV)
				{
					vertices[i0].uv = uvs[uv0];
					vertices[i1].uv = uvs[uv1];
					vertices[i2].uv = uvs[uv2];
				}
			}

			if (!currQuadblockGoodUV)
			{
				m_invalidQuadblocks.emplace_back(currQuadblockName, "Missing UVs.");
			}

			bool blockFetched = false;
			if (isQuadblock)
			{
				std::vector<std::string> token3 = Split(tokens[4], '/');
				int i3 = std::stoi(token3[0]) - 1;
				int ni3 = std::stoi(token3[2]) - 1;
				normalMap[currQuadblockName].push_back(normals[ni3]);
				vertices[i3].normal = normals[ni3];
				if (currQuadblockGoodUV)
				{
					int uv3 = std::stoi(token3[1]) - 1;
					vertices[i3].uv = uvs[uv3];
				}

				if (!quadMap.contains(currQuadblockName)) { quadMap[currQuadblockName] = std::vector<Quad>(); }
				quadMap[currQuadblockName].emplace_back(vertices[i0], vertices[i1], vertices[i2], vertices[i3]);
				blockFetched = quadMap[currQuadblockName].size() == 4;
			}
			else
			{
				if (!triMap.contains(currQuadblockName)) { triMap[currQuadblockName] = std::vector<Tri>(); }
				triMap[currQuadblockName].emplace_back(vertices[i0], vertices[i1], vertices[i2]);
				blockFetched = triMap[currQuadblockName].size() == 4;
			}

			if (blockFetched)
			{
				Vec3 averageNormal = Vec3();
				for (const Vec3& normal : normalMap[currQuadblockName])
				{
					averageNormal = averageNormal + normal;
				}
				averageNormal = averageNormal / averageNormal.Length();
				std::string material;
				if (materialMap.contains(currQuadblockName))
				{
					material = materialMap[currQuadblockName];
					m_materialToQuadblocks[material].push_back(m_quadblocks.size());
					if (!materials.contains(material))
					{
						materials.insert(material);
						m_materialToTexture[material] = Texture();
						m_propTerrain.SetDefaultValue(material, TerrainType::DEFAULT);
						m_propQuadFlags.SetDefaultValue(material, QuadFlags::DEFAULT);
						m_propDoubleSided.SetDefaultValue(material, false);
						m_propCheckpoints.SetDefaultValue(material, false);
						m_propTurboPads.SetDefaultValue(material, QuadblockTrigger::NONE);
						m_propCheckpointPathable.SetDefaultValue(material, true);
						m_propVisTreeTransparent.SetDefaultValue(material, false);
						m_propDrawOrderHigh.SetDefaultValue(material, static_cast<int>(0));
						m_propTerrain.RegisterMaterial(this);
						m_propQuadFlags.RegisterMaterial(this);
						m_propDoubleSided.RegisterMaterial(this);
						m_propCheckpoints.RegisterMaterial(this);
						m_propTurboPads.RegisterMaterial(this);
						m_propSpeedImpact.RegisterMaterial(this);
						m_propCheckpointPathable.RegisterMaterial(this);
						m_propVisTreeTransparent.RegisterMaterial(this);
						m_propDrawOrderHigh.RegisterMaterial(this);
					}
				}
				bool sameUVs = true;
				if (isQuadblock)
				{
					Quad& q0 = quadMap[currQuadblockName][0];
					Quad& q1 = quadMap[currQuadblockName][1];
					Quad& q2 = quadMap[currQuadblockName][2];
					Quad& q3 = quadMap[currQuadblockName][3];
					const Vec2& targetUV = q0.p[0].uv;
					for (size_t i = 0; i < 4; i++)
					{
						const Quad& q = quadMap[currQuadblockName][i];
						for (size_t j = 0; j < 4; j++)
						{
							if (q.p[j].uv != targetUV) { sameUVs = false; break; }
						}
						if (!sameUVs) { break; }
					}
					try
					{
						m_quadblocks.emplace_back(currQuadblockName, q0, q1, q2, q3, averageNormal, material, currQuadblockGoodUV, [this](const Quadblock& qb) { UpdateFilterRenderData(qb); });
						meshMap[currQuadblockName] = true;
					}
					catch (const QuadException& e)
					{
						ret = false;
						m_invalidQuadblocks.emplace_back(currQuadblockName, e.what());
					}
				}
				else
				{
					Tri& t0 = triMap[currQuadblockName][0];
					Tri& t1 = triMap[currQuadblockName][1];
					Tri& t2 = triMap[currQuadblockName][2];
					Tri& t3 = triMap[currQuadblockName][3];
					const Vec2& targetUV = t0.p[0].uv;
					for (size_t i = 0; i < 4; i++)
					{
						const Tri& t = triMap[currQuadblockName][i];
						for (size_t j = 0; j < 3; j++)
						{
							if (t.p[j].uv != targetUV) { sameUVs = false; break; }
						}
						if (!sameUVs) { break; }
					}
					try
					{
						m_quadblocks.emplace_back(currQuadblockName, t0, t1, t2, t3, averageNormal, material, currQuadblockGoodUV, [this](const Quadblock& qb) { UpdateFilterRenderData(qb); });
						meshMap[currQuadblockName] = true;
					}
					catch (const QuadException& e)
					{
						ret = false;
						m_invalidQuadblocks.emplace_back(currQuadblockName, e.what());
					}
				}
				if (sameUVs)
				{
					m_invalidQuadblocks.emplace_back(currQuadblockName, "Degenerated UV data.");
				}
			}
		}
	}
	file.close();

	m_showLogWindow = !m_invalidQuadblocks.empty();

	if (!materials.empty())
	{
		std::filesystem::path mtlPath = m_parentPath / (objFile.stem().string() + ".mtl");
		if (std::filesystem::exists(mtlPath))
		{
			std::ifstream mtl(mtlPath);
			std::string currMaterial;
			while (std::getline(mtl, line))
			{
				std::vector<std::string> tokens = Split(line);
				if (tokens.empty()) { continue; }

				const std::string& command = tokens[0];
				if (command == "newmtl") { currMaterial = tokens[1]; }
				else if (command == "map_Kd")
				{
					std::string imagePath = tokens[1];
					for (size_t i = 2; i < tokens.size(); i++) { imagePath += " " + tokens[i]; }
					std::filesystem::path materialPath = imagePath;
					if (!std::filesystem::exists(materialPath)) { materialPath = m_parentPath / materialPath.filename(); }
					if (std::filesystem::exists(materialPath))
					{
						m_materialToTexture[currMaterial] = Texture(materialPath);
					}
				}
			}
		}
	}

	if (ret)
	{
		for (const auto& [material, texture] : m_materialToTexture)
		{
			const bool semiTransparent = texture.IsSemiTransparent();
			m_propVisTreeTransparent.SetDefaultValue(material, semiTransparent);

			const std::filesystem::path& texPath = texture.GetPath();
			const std::vector<size_t>& quadblockIndexes = m_materialToQuadblocks[material];
			for (const size_t index : quadblockIndexes)
			{
				m_quadblocks[index].SetTexPath(texPath);
				m_quadblocks[index].SetVisTreeTransparent(semiTransparent);
			}
		}
	}

	if (quadblockCount != m_quadblocks.size())
	{
		m_showLogWindow = true;
		m_logMessage = "Error: number of meshes does not equal number of quadblocks.\n\nNumber of meshes found: " + std::to_string(quadblockCount) + "\nNumber of quadblocks: " + std::to_string(m_quadblocks.size());;
		m_logMessage += "\n\nThe following meshes are not a quadblock:\n\n";
		constexpr size_t QUADS_PER_LINE = 10;
		size_t invalidQuadblocks = 0;
		for (auto& [name, status] : meshMap)
		{
			if (status) { continue; }
			m_logMessage += name + ", ";
			if (((invalidQuadblocks + 1) % QUADS_PER_LINE) == 0) { m_logMessage += "\n"; }
			invalidQuadblocks++;
		}
		ret = false;
	}
	m_loaded = ret;

	if (m_loaded && isLevel)
	{
		std::filesystem::path presetFolder = m_parentPath / (m_name + "_presets");
		if (std::filesystem::is_directory(presetFolder))
		{
			for (const auto& entry : std::filesystem::directory_iterator(presetFolder))
			{
				const std::filesystem::path json = entry.path();
				if (json.has_extension() && json.extension() == ".json") { LoadPreset(json); }
			}
		}

		//Load preset models
		std::filesystem::path folderPath(Settings::m_lastOpenedModelFolder);
		if (std::filesystem::exists(folderPath) && std::filesystem::is_directory(folderPath))
		{
			for (const auto& entry : std::filesystem::directory_iterator(folderPath))
			{
				if (!entry.is_regular_file())
					continue;

				if (entry.path().extension() == ".ctrmodel")
				{
					ImportModel(entry.path());
				}
			}
		}
	}
	GenerateRenderLevData();
	GenerateBSP();
	return ret;
}





bool Level::SaveOBJ(const std::filesystem::path& objFile) 
{
	std::ofstream file(objFile);
	if (!file.is_open()) { return false; }

	// --- Collect all unique vertices, normals, UVs across all quadblocks ---
	std::vector<std::pair<Vec3, Color>> allVertices;
	std::vector<Vec3> allNormals;
	std::vector<Vec2> allUVs;

	// Maps for deduplication (using string keys for float precision safety)
	auto Vec3Key = [](const Vec3& v) {
		return std::to_string(v.x) + "," + std::to_string(v.y) + "," + std::to_string(v.z);
		};
	auto Vec2Key = [](const Vec2& v) {
		return std::to_string(v.x) + "," + std::to_string(v.y);
		};
	auto PointKey = [](const Vec3& pos) {
		return std::to_string(pos.x) + "," + std::to_string(pos.y) + "," + std::to_string(pos.z);
		};

	std::unordered_map<std::string, int> vertexIndexMap;  // 1-based
	std::unordered_map<std::string, int> normalIndexMap;  // 1-based
	std::unordered_map<std::string, int> uvIndexMap;      // 1-based

	// Per-quadblock face data: each face = list of (vi, uvi, ni) tuples
	struct FaceVertex { int vi, uvi, ni; };
	struct FaceData { std::vector<std::vector<FaceVertex>> faces; }; // each face is 3 or 4 verts
	std::vector<FaceData> quadblockFaces(m_quadblocks.size());

	auto GetOrAddVertex = [&](size_t quadblockIndex, int vertexSlot, const Vec3& pos, const Color& color) -> int {
		std::string key = std::to_string(quadblockIndex) + ":" + std::to_string(vertexSlot);
		auto it = vertexIndexMap.find(key);
		if (it != vertexIndexMap.end()) { return it->second; }
		int idx = (int)allVertices.size() + 1;
		allVertices.push_back({ pos, color });
		vertexIndexMap[key] = idx;
		return idx;
		};
	auto GetOrAddNormal = [&](const Vec3& n) -> int {
		std::string key = Vec3Key(n);
		auto it = normalIndexMap.find(key);
		if (it != normalIndexMap.end()) { return it->second; }
		int idx = (int)allNormals.size() + 1;
		allNormals.push_back(n);
		normalIndexMap[key] = idx;
		return idx;
		};
	auto GetOrAddUV = [&](const Vec2& uv) -> int {
		// Invert Y back to Blender convention before storing
		Vec2 blenderUV = { uv.x, 1.0f - uv.y };
		std::string key = Vec2Key(blenderUV);
		auto it = uvIndexMap.find(key);
		if (it != uvIndexMap.end()) { return it->second; }
		int idx = (int)allUVs.size() + 1;
		allUVs.push_back(blenderUV);
		uvIndexMap[key] = idx;
		return idx;
		};

	// --- Pass 1: gather all geometry into index buffers ---
	for (size_t qi = 0; qi < m_quadblocks.size(); qi++)
	{
		const Quadblock& qb = m_quadblocks[qi];
		FaceData& fd = quadblockFaces[qi];
		const Vertex* verts = qb.GetUnswizzledVertices();

		if (qb.IsQuadblock())
		{
			static constexpr int QUAD_FACES[4][4] = {
	{0, 3, 4, 1}, 
	{1, 4, 5, 2},
	{3, 6, 7, 4},
	{4, 7, 8, 5}
			};
			static constexpr int QUAD_UV_REMAP[4] = { 0, 2, 3, 1 }; 

			for (int f = 0; f < 4; f++)
			{
				const QuadUV& faceUVs = qb.GetQuadUV(f);
				std::vector<FaceVertex> face;
				for (int v = 0; v < 4; v++)
				{
					int slot = QUAD_FACES[f][v]; // p0..p8 index
					const Vertex& vert = verts[slot];
					face.push_back({
						GetOrAddVertex(qi, slot, vert.m_pos, vert.GetColor(true)),
						GetOrAddUV(faceUVs[QUAD_UV_REMAP[v]]),
						GetOrAddNormal(vert.m_normal)
						});
				}
				fd.faces.push_back(face);
			}
		}
		else // triblock
		{
			static constexpr int TRI_FACES[4][3] = {
	{3, 1, 0},  // tri {0,1,3} reversed
	{3, 4, 1},  // tri {1,4,3} reversed
	{4, 2, 1},  // tri {1,2,4} reversed
	{6, 4, 3}   // tri {3,4,6} reversed
			};

			// Which quadface UV to source from (same face index as parent quad)
			static constexpr int TRI_UV_FACE[4] = { 0, 0, 1, 2 };

			static constexpr int TRI_UV_REMAP[4][3] = {
				{2, 1, 0},  // face 0: correct
				{2, 3, 1},  // face 1: correct
				{2, 1, 0},  
				{2, 1, 0}   
			};

			for (int f = 0; f < 4; f++)
			{
				const QuadUV& faceUVs = qb.GetQuadUV(TRI_UV_FACE[f]);
				std::vector<FaceVertex> face;
				for (int v = 0; v < 3; v++)
				{
					int slot = TRI_FACES[f][v];
					const Vertex& vert = verts[slot];
					face.push_back({
						GetOrAddVertex(qi, slot, vert.m_pos, vert.GetColor(true)),
						GetOrAddUV(faceUVs[TRI_UV_REMAP[f][v]]),
						GetOrAddNormal(vert.m_normal)
						});
				}
				fd.faces.push_back(face);
			}
		}
	}

	// --- Write vertex positions ---
	file << std::fixed << std::setprecision(6);
	for (const auto& [pos, c] : allVertices)
	{
		file << "v " << pos.x << " " << pos.y << " " << pos.z
			<< " " << c.Red() << " " << c.Green() << " " << c.Blue() << "\n";
	}
	file << "\n";

	// --- Write UVs ---
	for (const Vec2& uv : allUVs)
	{
		file << "vt " << uv.x << " " << uv.y << "\n";
	}
	file << "\n";

	// --- Write normals ---
	for (const Vec3& n : allNormals)
	{
		file << "vn " << n.x << " " << n.y << " " << n.z << "\n";
	}
	file << "\n";

	// --- Write MTL reference ---
	std::string stem = objFile.stem().string();
	bool hasMaterials = !m_materialToTexture.empty();
	if (hasMaterials)
	{
		file << "mtllib " << stem << ".mtl\n\n";
	}

	// --- Write meshes (one per quadblock) ---
	for (size_t qi = 0; qi < m_quadblocks.size(); qi++)
	{
		const Quadblock& qb = m_quadblocks[qi];
		const FaceData& fd = quadblockFaces[qi];

		file << "o " << qb.GetName() << "\n";

		// Find this quadblock's material
		std::string material;
		for (const auto& [mat, indexes] : m_materialToQuadblocks)
		{
			for (size_t idx : indexes)
			{
				if (idx == qi) { material = mat; break; }
			}
			if (!material.empty()) { break; }
		}

		if (!material.empty())
		{
			file << "usemtl " << material << "\n";
		}

		file << "s off\n"; // smoothing group, standard Blender export

		for (const std::vector<FaceVertex>& face : fd.faces)
		{
			file << "f";
			for (const FaceVertex& fv : face)
			{
				file << " " << fv.vi << "/" << fv.uvi << "/" << fv.ni;
			}
			file << "\n";
		}
		file << "\n";
	}

	file.close();

	// --- Write MTL file ---
	if (hasMaterials)
	{
		std::filesystem::path mtlPath = objFile.parent_path() / (stem + ".mtl");
		std::ofstream mtl(mtlPath);
		if (mtl.is_open())
		{
			for (const auto& [material, texture] : m_materialToTexture)
			{
				mtl << "newmtl " << material << "\n";
				mtl << "Ka 1.000 1.000 1.000\n";
				mtl << "Kd 1.000 1.000 1.000\n";
				mtl << "Ks 0.000 0.000 0.000\n";
				mtl << "illum 1\n";

				const std::filesystem::path& texPath = texture.GetPath();
				if (!texPath.empty() && std::filesystem::exists(texPath))
				{
					mtl << "map_Kd " << texPath.filename().string() << "\n";
				}
				mtl << "\n";
			}
			mtl.close();
		}
	}

	return true;
}





bool Level::StartEmuIPC(const std::string& emulator)
{
	constexpr size_t PSX_RAM_SIZE = 0x800000;
	int pid = Process::GetPID(emulator);
	if (pid == Process::INVALID_PID || !Process::OpenMemoryMap(emulator + "_" + std::to_string(pid), PSX_RAM_SIZE)) { return false; }
	return true;
}

bool Level::HotReload(const std::string& levPath, const std::string& vrmPath, const std::string& emulator)
{
	bool vrmOnly = false;
	if (levPath.empty())
	{
		if (vrmPath.empty()) { return false; }
		vrmOnly = true;
	}

	if (!StartEmuIPC(emulator)) { return false; }

	constexpr size_t GAMEMODE_ADDR = 0x80096b20;
	constexpr uint32_t GAME_PAUSED = 0xF;
	if (Process::At<uint32_t>(GAMEMODE_ADDR) & GAME_PAUSED) { return false; }

	constexpr size_t VRAM_ADDR = 0x80200000;
	constexpr size_t RAM_ADDR = 0x80300000;
	constexpr size_t SIGNAL_ADDR = 0x8000C000;
	constexpr size_t SIGNAL_ADDR_VRAM_ONLY = 0x8000C004;
	constexpr int HOT_RELOAD_START = 1;
	constexpr int HOT_RELOAD_READY = 3;
	constexpr int HOT_RELOAD_EXEC = 4;

	if (!vrmOnly)
	{
		Process::At<int32_t>(SIGNAL_ADDR) = HOT_RELOAD_START;
		while (Process::At<volatile int32_t>(SIGNAL_ADDR) != HOT_RELOAD_READY) {}
	}
	if (!vrmPath.empty())
	{
		std::vector<uint8_t> vrm;
		ReadBinaryFile(vrm, vrmPath);
		for (size_t i = 0; i < vrm.size(); i++) { Process::At<uint8_t>(VRAM_ADDR + i) = vrm[i]; }
	}

	if (!levPath.empty())
	{
		std::vector<uint8_t> lev;
		ReadBinaryFile(lev, levPath);
		for (size_t i = 0; i < lev.size(); i++) { Process::At<uint8_t>(RAM_ADDR + i) = lev[i]; }
	}

	if (vrmOnly) { Process::At<int32_t>(SIGNAL_ADDR_VRAM_ONLY) = 1; }
	else { Process::At<int32_t>(SIGNAL_ADDR) = HOT_RELOAD_EXEC; }

	return true;
}

bool Level::SaveGhostData(const std::string& emulator, const std::filesystem::path& path)
{
	constexpr size_t SIGNAL_ADDR = 0x8000C008;
	if (!StartEmuIPC(emulator) || Process::At<int32_t>(SIGNAL_ADDR) == 0) { return false; }

	std::vector<uint8_t> data;
	constexpr size_t GHOST_SIZE_ADDR = 0x80270038;
	constexpr size_t GHOST_DATA_ADDR = 0x8027003C;

	size_t fileSize = static_cast<size_t>(Process::At<uint32_t>(GHOST_SIZE_ADDR));
	if (fileSize != GHOST_DATA_FILESIZE) { return false; }

	data.resize(fileSize);
	for (size_t i = 0; i < data.size(); i++) { data[i] = Process::At<uint8_t>(GHOST_DATA_ADDR + i); }
	Process::At<int32_t>(SIGNAL_ADDR) = 0;

	std::ofstream file(path, std::ios::binary);
	Write(file, data.data(), data.size() * sizeof(uint8_t));
	file.close();
	return true;
}

bool Level::SetGhostData(const std::filesystem::path& path, bool tropy)
{
	std::vector<uint8_t> data;
	ReadBinaryFile(data, path);
	if (data.size() != GHOST_DATA_FILESIZE) { return false; }

	if (tropy) { m_tropyGhost.resize(GHOST_DATA_FILESIZE); }
	else { m_oxideGhost.resize(GHOST_DATA_FILESIZE); }
	memcpy(tropy ? m_tropyGhost.data() : m_oxideGhost.data(), data.data(), data.size());
	return true;
}

bool Level::UpdateVRM()
{
	std::vector<Texture*> textures;
	std::vector<std::tuple<Texture*, Texture*>> copyTextureAttributes;
	for (auto& [material, texture] : m_materialToTexture)
	{
		bool foundEqual = false;
		for (Texture* addedTexture : textures)
		{
			if (texture == *addedTexture)
			{
				copyTextureAttributes.push_back({addedTexture, &texture});
				foundEqual = true;
				break;
			}
		}
		if (foundEqual) { continue; }
		textures.push_back(&texture);
	}

	for (const AnimTexture& animTex : m_animTextures)
	{
		const std::vector<AnimTextureFrame>& animFrames = animTex.GetFrames();
		const std::vector<Texture>& animTextures = animTex.GetTextures();
		for (const AnimTextureFrame& frame : animFrames)
		{
			bool foundEqual = false;
			Texture* texture = const_cast<Texture*>(&animTextures[frame.textureIndex]);
			for (Texture* addedTexture : textures)
			{
				if (*texture == *addedTexture)
				{
					copyTextureAttributes.push_back({addedTexture, texture});
					foundEqual = true;
					break;
				}
			}
			if (foundEqual) { continue; }
			textures.push_back(texture);
		}
	}

	// Extract textures from imported models
	m_modelTexturesInVRAM.clear();
	for (const auto& [modelName, ctrmodelData] : m_importedModels)
	{
		const SH::CtrModel* ctrHeader = reinterpret_cast<const SH::CtrModel*>(ctrmodelData.data());

		// Skip if no texture section
		if (ctrHeader->textureDataOffset == 0) { continue; }

		// Parse texture section
		const SH::TextureSectionHeader* texSection =
			reinterpret_cast<const SH::TextureSectionHeader*>(ctrmodelData.data() + ctrHeader->textureDataOffset);

		if (texSection->numTextures == 0) { continue; }

		// Offset array follows header
		const uint32_t* texOffsets = reinterpret_cast<const uint32_t*>(texSection + 1);

		for (uint32_t i = 0; i < texSection->numTextures; i++)
		{
			const SH::TextureDataHeader* texData =
				reinterpret_cast<const SH::TextureDataHeader*>(ctrmodelData.data() + texOffsets[i]);

			ModelTextureForVRM modelTex;
			modelTex.modelName = modelName;
			modelTex.textureIndex = i;
			modelTex.width = texData->width;
			modelTex.height = texData->height;
			modelTex.bpp = texData->bpp;
			modelTex.blendMode = texData->blendMode;
			modelTex.origPageX = texData->origPageX;
			modelTex.origPageY = texData->origPageY;
			modelTex.origPalX = texData->origPalX;
			modelTex.origPalY = texData->origPalY_lo | (texData->origPalY_hi << 8);
			modelTex.originU = texData->originU;
			modelTex.originV = texData->originV;

			// Calculate pixel data size
			size_t pixelDataSize = 0;
			size_t paletteSize = 0;
			if (texData->bpp == 0) // 4-bit
			{
				pixelDataSize = ((texData->width + 1) / 2) * texData->height;
				paletteSize = 16;
			}
			else if (texData->bpp == 1) // 8-bit
			{
				pixelDataSize = texData->width * texData->height;
				paletteSize = 256;
			}
			else // 16-bit
			{
				pixelDataSize = texData->width * texData->height * 2;
				paletteSize = 0;
			}

			// Copy pixel data
			const uint8_t* pixelStart = reinterpret_cast<const uint8_t*>(texData + 1);
			modelTex.pixelData.assign(pixelStart, pixelStart + pixelDataSize);

			// Copy palette data (if indexed)
			if (paletteSize > 0)
			{
				const uint16_t* paletteStart = reinterpret_cast<const uint16_t*>(pixelStart + pixelDataSize);
				modelTex.palette.assign(paletteStart, paletteStart + paletteSize);
			}

			m_modelTexturesInVRAM.push_back(std::move(modelTex));
		}
	}

	m_vrm = PackVRM(textures, m_modelTexturesInVRAM.empty() ? nullptr : &m_modelTexturesInVRAM);
	if (m_vrm.empty()) { return false; }

	for (auto& [from, to] : copyTextureAttributes)
	{
		to->CopyVRAMAttributes(*from);
	}

	return true;
}

std::vector<uint16_t> Level::ReadRawVRAM(std::filesystem::path vrmPath)
{
	std::vector<uint16_t> vram(1024 * 512, 0); 

	if (std::filesystem::exists(vrmPath))
	{
		std::ifstream vrmFile(vrmPath, std::ios::binary);

		// Read the raw file into temporary memory
		vrmFile.seekg(0, std::ios::end);
		size_t vrmSize = vrmFile.tellg();
		vrmFile.seekg(0, std::ios::beg);

		std::vector<uint8_t> rawVrmData(vrmSize);
		vrmFile.read(reinterpret_cast<char*>(rawVrmData.data()), vrmSize);
		vrmFile.close();

		const uint8_t* pVrm = rawVrmData.data();
		uint32_t vrmMagic;
		memcpy(&vrmMagic, pVrm, sizeof(uint32_t));
		pVrm += sizeof(uint32_t);

		// If magic is 0x20, we have a multi-block VRM (Standard for this level format)
		if (vrmMagic == 0x20) {
			for (int block = 0; block < 2; block++) {
				PSX::VRMHeader blockHead;
				memcpy(&blockHead, pVrm, sizeof(PSX::VRMHeader));
				pVrm += sizeof(PSX::VRMHeader);

				for (size_t y = 0; y < blockHead.height; y++) {
					// Use the absolute coordinates provided in the VRM header
					size_t vramIdx = (blockHead.y + y) * 1024 + blockHead.x;
					size_t rowByteSize = blockHead.width * sizeof(uint16_t);

					if (vramIdx + blockHead.width <= vram.size()) {
						memcpy(&vram[vramIdx], pVrm, rowByteSize);
					}
					pVrm += rowByteSize;
				}
			}
		}
	}
	return vram;
}

bool Level::UpdateAnimTextures(float deltaTime)
{
	bool changed = false;
	if (m_animTextures.size() != m_lastAnimTextureCount)
	{
		m_lastAnimTextureCount = m_animTextures.size();
		changed = true;
	}

	for (AnimTexture& animTex : m_animTextures)
	{
		if (animTex.AdvanceRender(deltaTime)) { changed = true; }
	}

	return changed;
}

void Level::InitModels(Renderer& renderer)
{
	m_models[LevelModels::LEVEL] = renderer.CreateModel();
	m_models[LevelModels::LEVEL]->SetRenderCondition([]() { return GuiRenderSettings::showLevel; });

	m_models[LevelModels::BSP] = m_models[LevelModels::LEVEL]->AddModel();
	m_models[LevelModels::BSP]->SetRenderCondition([]() { return GuiRenderSettings::showBspRectTree; });

	m_models[LevelModels::SPAWN] = m_models[LevelModels::LEVEL]->AddModel();
	m_models[LevelModels::SPAWN]->SetRenderCondition([]() { return GuiRenderSettings::showStartpoints; });

	m_models[LevelModels::CHECKPOINT] = m_models[LevelModels::LEVEL]->AddModel();
	m_models[LevelModels::CHECKPOINT]->SetRenderCondition([]() { return GuiRenderSettings::showCheckpoints; });

	m_models[LevelModels::SELECTED] = m_models[LevelModels::LEVEL]->AddModel();

	m_models[LevelModels::MULTI_SELECTED] = m_models[LevelModels::LEVEL]->AddModel();
	m_models[LevelModels::MULTI_SELECTED]->SetRenderCondition([]() { return GuiRenderSettings::showVisTree; });

	m_models[LevelModels::FILTER] = m_models[LevelModels::LEVEL]->AddModel();
	m_models[LevelModels::FILTER]->SetRenderCondition([]() { return GuiRenderSettings::filterActive; });

	m_models[LevelModels::SKYBOX] = m_models[LevelModels::LEVEL]->AddModel();
	m_models[LevelModels::SKYBOX]->SetRenderCondition([]() { return GuiRenderSettings::showSkybox; });

	m_models[LevelModels::BOT] = m_models[LevelModels::LEVEL]->AddModel();
	m_models[LevelModels::BOT]->SetRenderCondition([]() { return GuiRenderSettings::showBots; });

	m_models[LevelModels::INSTANCES] = m_models[LevelModels::LEVEL]->AddModel();
	m_models[LevelModels::INSTANCES]->SetRenderCondition([]() { return GuiRenderSettings::showInstances; });
}

void Level::GenerateRenderLevData()
{
	if (!m_models[LevelModels::LEVEL] || !m_models[LevelModels::FILTER]) { return; }

	std::vector<Primitive> levTriangles;
	std::vector<Primitive> filterTriangles;
	levTriangles.reserve(m_quadblocks.size() * 8);
	filterTriangles.reserve(m_quadblocks.size() * 8);

	auto CountPrimitiveTriangles = [](const std::vector<Primitive>& primitives)
		{
			size_t count = 0;
			for (const Primitive& primitive : primitives) { count += (primitive.type == PrimitiveType::QUAD) ? 2 : 1; }
			return count;
		};

	size_t triangleOffset = 0;
	for (Quadblock& qb : m_quadblocks)
	{
		std::vector<Primitive> qbTriangles = qb.ToGeometry(false);
		if (qbTriangles.empty()) { continue; }

		qb.SetRenderPrimitiveIndex(triangleOffset);
		std::vector<Primitive> qbFilterTriangles = qb.ToGeometry(true);
		levTriangles.insert(levTriangles.end(), qbTriangles.begin(), qbTriangles.end());
		filterTriangles.insert(filterTriangles.end(), qbFilterTriangles.begin(), qbFilterTriangles.end());
		const size_t qbTriCount = CountPrimitiveTriangles(qbTriangles);
		triangleOffset += qbTriCount;
	}

	m_models[LevelModels::LEVEL]->GetMesh().SetGeometry(levTriangles, Mesh::RenderFlags::AllowPointRender | Mesh::RenderFlags::QuadblockLod, Mesh::ShaderFlags::None);
	m_models[LevelModels::FILTER]->GetMesh().SetGeometry(filterTriangles,
		Mesh::RenderFlags::DrawWireframe | Mesh::RenderFlags::DrawBackfaces | Mesh::RenderFlags::ForceDrawOnTop | Mesh::RenderFlags::DrawLinesAA | Mesh::RenderFlags::DontOverrideRenderFlags | Mesh::RenderFlags::ThickLines | Mesh::RenderFlags::QuadblockLod,
		Mesh::ShaderFlags::DiscardZeroColor);
}

void Level::UpdateAnimationRenderData()
{
	if (!m_models[LevelModels::LEVEL]) { return; }

	for (const AnimTexture& animTex : m_animTextures)
	{
		if (!animTex.IsPopulated()) { continue; }
		const std::vector<Texture>& textures = animTex.GetTextures();
		const AnimTextureFrame& frame = animTex.GetRenderFrame();
		for (size_t qbIndex : animTex.GetQuadblockIndexes())
		{
			Quadblock& qb = m_quadblocks[qbIndex];
			const std::array<QuadUV, NUM_FACES_QUADBLOCK + 1>& uvs = frame.uvs;
			const size_t basePrimitiveIndex = qb.GetRenderPrimitiveIndex();
			if (basePrimitiveIndex == RENDER_INDEX_NONE) { continue; }

			const std::filesystem::path texturePath = textures[frame.textureIndex].GetPath();
			std::vector<Primitive> qbTriangles = qb.ToGeometry(false, &uvs, &texturePath);
			size_t primitiveIndex = basePrimitiveIndex;
			for (const Primitive& primitive : qbTriangles)
			{
				primitiveIndex = m_models[LevelModels::LEVEL]->GetMesh().UpdatePrimitive(primitive, primitiveIndex);
			}
		}
	}
}

void Level::UpdateFilterRenderData(const Quadblock& qb)
{
	if (!m_models[LevelModels::FILTER]) { return; }

	const size_t basePrimitiveIndex = qb.GetRenderPrimitiveIndex();
	if (basePrimitiveIndex == RENDER_INDEX_NONE) { return; }

	std::vector<Primitive> qbFilterTriangles = qb.ToGeometry(true);
	size_t primitiveIndex = basePrimitiveIndex;
	for (const Primitive& primitive : qbFilterTriangles)
	{
		primitiveIndex = m_models[LevelModels::FILTER]->GetMesh().UpdatePrimitive(primitive, primitiveIndex);
	}
}

void Level::GenerateRenderBspData()
{
	if (!m_models[LevelModels::BSP]) { return; }

	struct NodeDepth
	{
		const BSP* node;
		int depth;
	};
	std::vector<Primitive> triangles;
	std::vector<NodeDepth> stack;
	stack.push_back({&m_bsp, 0});
	GuiRenderSettings::bspTreeMaxDepth = 0;
	while (!stack.empty())
	{
		const NodeDepth entry = stack.back();
		stack.pop_back();
		if (!entry.node) { continue; }

		if (GuiRenderSettings::bspTreeMaxDepth < entry.depth)
		{
			GuiRenderSettings::bspTreeMaxDepth = entry.depth;
		}

		const bool drawDepth = (GuiRenderSettings::bspTreeTopDepth <= entry.depth && GuiRenderSettings::bspTreeBottomDepth >= entry.depth);
		if (drawDepth)
		{
			const Color c = Color(entry.depth * 30.0, 1.0, 1.0);
			std::vector<Primitive> nodeTriangles = entry.node->GetBoundingBox().ToGeometry();
			for (Primitive& primitive : nodeTriangles)
			{
				for (unsigned i = 0; i < primitive.pointCount; i++) { primitive.p[i].color = c; }
				triangles.push_back(primitive);
			}
		}

		if (entry.node->GetLeftChildren() != nullptr)
		{
			stack.push_back({entry.node->GetLeftChildren(), entry.depth + 1});
		}
		if (entry.node->GetRightChildren() != nullptr)
		{
			stack.push_back({entry.node->GetRightChildren(), entry.depth + 1});
		}
	}

	m_models[LevelModels::BSP]->GetMesh().SetGeometry(triangles, Mesh::RenderFlags::DrawWireframe | Mesh::RenderFlags::DontOverrideRenderFlags);
}

void Level::UpdateRenderCheckpointData()
{
	Model* checkpointModel = m_models[LevelModels::CHECKPOINT];
	if (!checkpointModel) { return; }

	checkpointModel->ClearModels();
	if (m_checkpoints.empty())
	{
		checkpointModel->GetMesh().Clear();
		return;
	}

	std::vector<Primitive> checkTriangles;
	checkTriangles.reserve(m_checkpoints.size() * 8);
	std::unordered_set<int> selectedCheckpointIndexes;
	for (size_t index : m_rendererSelectedQuadblockIndexes)
	{
		int checkpointIndex = m_quadblocks[index].GetCheckpoint();
		selectedCheckpointIndexes.insert(checkpointIndex);
	}

	constexpr float labelHeightOffset = 1.5f;
	for (const Checkpoint& e : m_checkpoints)
	{
		bool selected = selectedCheckpointIndexes.contains(e.GetIndex());
		const Color& c = selected ? GuiRenderSettings::selectedCheckpointColor : e.GetColor();
		Vertex v = Vertex(Point(e.GetPos().x, e.GetPos().y, e.GetPos().z, c.r, c.g, c.b));
		const std::vector<Primitive> tris = v.ToGeometry();
		checkTriangles.insert(checkTriangles.end(), tris.begin(), tris.end());

		Model* label = checkpointModel->AddModel();
		label->GetMesh().SetGeometry("CP " + std::to_string(e.GetIndex()), Text3D::Align::CENTER, Color(c.r, c.g, c.b, static_cast<unsigned char>(255u)));
		Vec3 labelPos = e.GetPos();
		labelPos.y += labelHeightOffset;
		label->SetPosition(labelPos);
	}

	checkpointModel->GetMesh().SetGeometry(checkTriangles, Mesh::RenderFlags::DrawBackfaces | Mesh::RenderFlags::DontOverrideRenderFlags);
}


void Level::UpdateRenderBotData()
{
	Model* botModel = m_models[LevelModels::BOT];
	if (!botModel) { return; }
	botModel->ClearModels();

	// Check if any path has nodes at all
	bool anyNodes = false;
	for (const BotPath& path : m_botPaths)
		if (path.GetNodeCount() > 0) { anyNodes = true; break; }

	if (!anyNodes)
	{
		botModel->GetMesh().Clear();
		return;
	}

	// One fixed color per path (left, middle, right)
	static const Color pathColors[3] =
	{
		Color(0.86f, 0.31f, 0.31f), // left   red
		Color(0.31f, 0.78f, 0.31f), // mid    green
		Color(0.31f, 0.51f, 0.86f), // right  blue
	};

	constexpr float labelHeightOffset = 1.5f;
	std::vector<Primitive> botTriangles;

	for (int pathIndex = 0; pathIndex < 3; pathIndex++)
	{
		const BotPath& path = m_botPaths[pathIndex];
		const Color& c = pathColors[pathIndex % 3];

		for (size_t nodeIndex = 0; nodeIndex < path.GetNodeCount(); nodeIndex++)
		{
			const BotNode& node = path.GetNode(nodeIndex);
			const Vec3& pos = node.GetPos();

			Vertex v = Vertex(Point(pos.x, pos.y, pos.z, c.r, c.g, c.b));
			const std::vector<Primitive> tris = v.ToGeometry();
			botTriangles.insert(botTriangles.end(), tris.begin(), tris.end());

			Model* label = botModel->AddModel();
			label->GetMesh().SetGeometry(
				std::to_string(nodeIndex),
				Text3D::Align::CENTER,
				Color(c.r, c.g, c.b, 255u)
			);
			Vec3 labelPos = pos;
			labelPos.y += labelHeightOffset;
			label->SetPosition(labelPos);
		}
	}

	botModel->GetMesh().SetGeometry(
		botTriangles,
		Mesh::RenderFlags::DrawBackfaces | Mesh::RenderFlags::DontOverrideRenderFlags
	);
}


// Decode PSX texture pixel data (4-bit/8-bit indexed or 16-bit RGBA5551) to RGBA PNG
// Returns the path on success, empty string on failure
static std::string DecodePsxTextureToPng(
    const uint8_t* pixelData, size_t pixelDataSize,
    const uint16_t* palette, size_t paletteSize,
    uint16_t width, uint16_t height, uint8_t bpp,
    const std::string& outputPath)
{
    if (pixelData == nullptr || width == 0 || height == 0 || width > 1024 || height > 1024) { return {}; }

    size_t expectedSize;
    if (bpp == 0) expectedSize = ((width + 1) / 2) * height;
    else if (bpp == 1) expectedSize = width * height;
    else if (bpp == 2) expectedSize = width * height * 2;
    else { return {}; }

    if (pixelDataSize < expectedSize) { return {}; }

    std::vector<uint8_t> rgba(width * height * 4);

    auto convert5551 = [](uint16_t val) -> std::array<uint8_t, 4> {
        // GodotCTR: r=bits0-4, g=bits5-9, b=bits10-14, alpha always 1 except all-black=transparent
        uint8_t r = static_cast<uint8_t>(((val >> 0) & 0x1F) * 8);
        uint8_t g = static_cast<uint8_t>(((val >> 5) & 0x1F) * 8);
        uint8_t b = static_cast<uint8_t>(((val >> 10) & 0x1F) * 8);
        uint8_t a = 255;
        if (r == 0 && g == 0 && b == 0) a = 0;
        return { r, g, b, a };
    };

    std::vector<std::array<uint8_t, 4>> clut;
    if (palette && paletteSize > 0)
    {
        clut.reserve(paletteSize);
        for (size_t i = 0; i < paletteSize; i++)
            clut.push_back(convert5551(palette[i]));
    }

    for (uint16_t y = 0; y < height; y++)
    {
        for (uint16_t x = 0; x < width; x++)
        {
            size_t dstIdx = (size_t(y) * width + x) * 4;
            std::array<uint8_t, 4> color = { 0, 0, 0, 255 };

            if (bpp == 0)
            {
                size_t srcIdx = y * ((width + 1) / 2) + x / 2;
                uint8_t byte = pixelData[srcIdx];
                int nibble = (x % 2 == 0) ? (byte & 0x0F) : ((byte >> 4) & 0x0F);
                if (nibble < (int)clut.size()) color = clut[nibble];
            }
            else if (bpp == 1)
            {
                size_t srcIdx = size_t(y) * width + x;
                uint8_t index = pixelData[srcIdx];
                if (index < clut.size()) color = clut[index];
            }
            else if (bpp == 2)
            {
                size_t srcIdx = (size_t(y) * width + x) * 2;
                uint16_t val = pixelData[srcIdx] | (static_cast<uint16_t>(pixelData[srcIdx + 1]) << 8);
                color = convert5551(val);
            }

            rgba[dstIdx + 0] = color[0];
            rgba[dstIdx + 1] = color[1];
            rgba[dstIdx + 2] = color[2];
            rgba[dstIdx + 3] = color[3];
        }
    }

    if (stbi_write_png(outputPath.c_str(), width, height, 4, rgba.data(), width * 4))
        return outputPath;
    return {};
}


// Parse .ctrmodel binary data into renderable primitives (positions, colors, UVs, normals)
// Modeled after GodotCTR's modelLoader.gd: getFinalTriVerts()
// If cacheDir is non-empty, embedded textures are decoded to PNG files in that directory
// modelName is used to uniquify texture filenames, preventing cache collision between models
static std::vector<Primitive> ParseCtrModelGeometry(const std::vector<uint8_t>& data, const std::string& cacheDir = {}, const std::string& modelName = {})
{
	std::vector<Primitive> result;

	if (data.size() < sizeof(SH::CtrModel)) { return result; }

	const SH::CtrModel* ctrHeader = reinterpret_cast<const SH::CtrModel*>(data.data());
	const PSX::Model* psxModel = reinterpret_cast<const PSX::Model*>(data.data() + ctrHeader->modelOffset);

	if (psxModel->numHeaders == 0 || psxModel->offHeaders == 0) { return result; }

	// Use the first header (highest LOD)
	PSX::ModelHeader modelHeader;
	memcpy(&modelHeader, data.data() + psxModel->offHeaders, sizeof(PSX::ModelHeader));

	if (modelHeader.offCommandList == 0 || modelHeader.offFrameData == 0) { return result; }

	// --- Per-submodel scale ---
	// GodotCTR uses int16 * 0.0008 = int16 / 1250, but that's calibrated for
	// GodotCTR's map scale (0.012). Our editor uses FP_ONE_GEO (1/64 ≈ 0.015625).
	// The ratio (1/64)/0.012 = 1.302 gives us: 1/1250 * 1.302 = 1/960.
	Vec3 modelScale(
		modelHeader.scale.x * (1.0f / 960.0f),
		modelHeader.scale.y * (1.0f / 960.0f),
		modelHeader.scale.z * (1.0f / 960.0f)
	);

	// --- Read command list ---
	const uint8_t* cmdBase = data.data() + modelHeader.offCommandList;
	uint32_t unkNum;
	memcpy(&unkNum, cmdBase, sizeof(uint32_t));
	const PSX::InstDrawCommand* commands = reinterpret_cast<const PSX::InstDrawCommand*>(cmdBase + 4);

	size_t numCommands = 0;
	for (; commands[numCommands].command != 0xFFFFFFFF; numCommands++) {}

	if (numCommands == 0 || numCommands > 10000) { return result; }

	// --- Read frame data ---
	const PSX::ModelFrame* modelFrame = reinterpret_cast<const PSX::ModelFrame*>(data.data() + modelHeader.offFrameData);

	// GodotCTR frame origin: readVector(data, 0.00390625) = int16 * 1/256
	Vec3 frameOrigin(
		modelFrame->pos.x * (1.0f / 256.0f),
		modelFrame->pos.y * (1.0f / 256.0f),
		modelFrame->pos.z * (1.0f / 256.0f)
	);

	// Vertex data: uint8 × 3 per vertex (GodotCTR: data.get_u8())
	const uint8_t* vertData = reinterpret_cast<const uint8_t*>(reinterpret_cast<const uint8_t*>(modelFrame) + modelFrame->vertexOffset);

	// --- Parse embedded textures ---
	struct TexKey {
		uint8_t pageX, pageY, palX, bpp;
		uint16_t palY;
		bool operator==(const TexKey& o) const { return pageX == o.pageX && pageY == o.pageY && palX == o.palX && palY == o.palY && bpp == o.bpp; }
	};
	struct TexKeyHash {
		size_t operator()(const TexKey& k) const {
			return (size_t(k.pageX) << 0) ^ (size_t(k.pageY) << 4) ^ (size_t(k.palX) << 8) ^ (size_t(k.palY) << 16) ^ (size_t(k.bpp) << 32);
		}
	};
	std::unordered_map<TexKey, std::string, TexKeyHash> texKeyToPath;

	if (ctrHeader->textureDataOffset != 0 && !cacheDir.empty())
	{
		std::filesystem::create_directories(cacheDir);
		const uint8_t* texBase = data.data() + ctrHeader->textureDataOffset;
		SH::TextureSectionHeader texSection;
		memcpy(&texSection, texBase, sizeof(SH::TextureSectionHeader));

		if (texSection.numTextures > 0 && texSection.numTextures < 256)
		{
			for (uint32_t t = 0; t < texSection.numTextures; t++)
			{
				uint32_t texOffset;
				memcpy(&texOffset, texBase + sizeof(SH::TextureSectionHeader) + t * sizeof(uint32_t), sizeof(uint32_t));

				// Validate offset is within data
				if (texOffset + sizeof(SH::TextureDataHeader) > data.size()) { continue; }

				const uint8_t* texDataPtr = data.data() + texOffset;

				SH::TextureDataHeader texHeader;
				memcpy(&texHeader, texDataPtr, sizeof(SH::TextureDataHeader));

				// Validate dimensions
				if (texHeader.width == 0 || texHeader.height == 0 || texHeader.width > 1024 || texHeader.height > 1024) { continue; }

				const uint8_t* pixData = texDataPtr + sizeof(SH::TextureDataHeader);

				size_t pixSize;
				if (texHeader.bpp == 0) pixSize = ((texHeader.width + 1) / 2) * texHeader.height;
				else if (texHeader.bpp == 1) pixSize = texHeader.width * texHeader.height;
				else if (texHeader.bpp == 2) pixSize = texHeader.width * texHeader.height * 2;
				else { continue; }

				// Validate total texture data fits in buffer
				size_t palByteSize = (texHeader.bpp == 0) ? (16 * sizeof(uint16_t)) : (texHeader.bpp == 1) ? (256 * sizeof(uint16_t)) : 0;
				if (texOffset + sizeof(SH::TextureDataHeader) + pixSize + palByteSize > data.size()) { continue; }

				const uint16_t* palData = nullptr;
				size_t palSize = 0;
				if (texHeader.bpp < 2)
				{
					palData = reinterpret_cast<const uint16_t*>(pixData + pixSize);
					palSize = (texHeader.bpp == 0) ? 16 : 256;
				}

				uint16_t palY = texHeader.origPalY_lo | (static_cast<uint16_t>(texHeader.origPalY_hi) << 8);
				std::string pngPath = cacheDir + "/" + modelName + "_tex_" + std::to_string(t) + ".png";

				if (DecodePsxTextureToPng(pixData, pixSize, palData, palSize,
					texHeader.width, texHeader.height, texHeader.bpp,
					pngPath) == pngPath)
				{
					TexKey key{ texHeader.origPageX, texHeader.origPageY, texHeader.origPalX, texHeader.bpp, palY };
					texKeyToPath[key] = pngPath;
				}
			}
		}
	}

	// --- Read texture layouts ---
	std::vector<Vec2> texUVs;
	std::vector<std::string> texIdxToPath;
	if (modelHeader.offTexLayout != 0)
	{
		const uint32_t* texLayoutPtrs = reinterpret_cast<const uint32_t*>(data.data() + modelHeader.offTexLayout);
		uint32_t maxTexIdx = 0;
		for (size_t ci = 0; ci < numCommands; ci++)
		{
			if (commands[ci].texCoordIndex > maxTexIdx) { maxTexIdx = commands[ci].texCoordIndex; }
		}
		if (maxTexIdx > 0)
		{
			texUVs.resize(maxTexIdx * 4);
			texIdxToPath.resize(maxTexIdx);
		}
		for (uint32_t ti = 0; ti < maxTexIdx; ti++)
		{
			PSX::TextureLayout layout;
			memcpy(&layout, data.data() + texLayoutPtrs[ti], sizeof(PSX::TextureLayout));

			// Raw UVs (0-255 from PSX, stored temporarily as floats)
			float rawU[4] = { layout.u0, layout.u1, layout.u2, layout.u3 };
			float rawV[4] = { layout.v0, layout.v1, layout.v2, layout.v3 };

			// Normalize: map the bounding box of the 4 UVs to 0-1 range
			// (GodotCTR's normalizeUV: ((raw - min) / (max - min)) * 255, then /255)
			float minU = 255, minV = 255, maxU = 0, maxV = 0;
			for (int i = 0; i < 4; i++)
			{
				if (rawU[i] < minU) minU = rawU[i];
				if (rawV[i] < minV) minV = rawV[i];
				if (rawU[i] > maxU) maxU = rawU[i];
				if (rawV[i] > maxV) maxV = rawV[i];
			}
			float rangeU = maxU - minU;
			float rangeV = maxV - minV;
			if (rangeU < 1.0f) rangeU = 1.0f;
			if (rangeV < 1.0f) rangeV = 1.0f;

			for (int i = 0; i < 4; i++)
			{
				float nu = (rawU[i] - minU) / rangeU;
				float nv = (rawV[i] - minV) / rangeV;
				texUVs[ti * 4 + i] = Vec2(
					std::clamp(nu, 0.0f, 1.0f),
					std::clamp(nv, 0.0f, 1.0f)
				);
			}

			// Match texture by VRAM coordinates (texpage + CLUT)
			TexKey key{ layout.texPage.x, layout.texPage.y, layout.clut.x, layout.texPage.texpageColors, layout.clut.y };
			auto kit = texKeyToPath.find(key);
			if (kit != texKeyToPath.end())
				texIdxToPath[ti] = kit->second;
		}
	}

	// --- Read colors ---
	std::vector<Color> colors;
	if (modelHeader.offColors != 0)
	{
		const uint32_t* colorData = reinterpret_cast<const uint32_t*>(data.data() + modelHeader.offColors);
		uint32_t maxColIdx = 0;
		for (size_t ci = 0; ci < numCommands; ci++)
		{
			if (commands[ci].colorCoordIndex > maxColIdx) { maxColIdx = commands[ci].colorCoordIndex; }
		}
		colors.resize(maxColIdx + 1);
		for (uint32_t ci = 0; ci <= maxColIdx; ci++)
		{
			uint32_t rgba = colorData[ci];
			unsigned char r = (rgba >> 0) & 0xFF;
			unsigned char g = (rgba >> 8) & 0xFF;
			unsigned char b = (rgba >> 16) & 0xFF;
			unsigned char a = (rgba >> 24) & 0xFF;
			PSX::Color psxC = {};
			psxC.r = r; psxC.g = g; psxC.b = b; psxC.a = a;
			colors[ci] = ConvertColor(psxC);
		}
	}

	// --- Count unique vertices ---
	int numVerts = 0;
	for (size_t ci = 0; ci < numCommands; ci++)
	{
		if (!commands[ci].readNextVertFromStackIndexFlag)
			numVerts++;
	}

	// --- Decode vertices using GodotCTR formula ---
	// vfixed[i].x = ((srcVert.x / 255.0) + offset.x) * scale.x
	// vfixed[i].y = ((srcVert.z / 255.0) + offset.y) * scale.y  (Y/Z swizzle!)
	// vfixed[i].z = ((srcVert.y / 255.0) + offset.z) * scale.z
	struct VertData {
		Vec3 pos;
		Color color;
		Vec2 uv;
	};
	std::vector<VertData> vfixed;
	vfixed.reserve(numVerts);
	for (int i = 0; i < numVerts; i++)
	{
		const uint8_t* src = vertData + i * 3;
		Vec3 pos;
		pos.x = ((src[0] / 255.0f) + frameOrigin.x) * modelScale.x;
		pos.y = ((src[2] / 255.0f) + frameOrigin.y) * modelScale.y;
		pos.z = ((src[1] / 255.0f) + frameOrigin.z) * modelScale.z;

		// GodotCTR applies * Vector3(-1, 1, -1) to all vertices
		pos.x = -pos.x;
		pos.z = -pos.z;

		VertData vd;
		vd.pos = pos;
		vd.color = Color(static_cast<unsigned char>(128), static_cast<unsigned char>(128), static_cast<unsigned char>(128));
		vd.uv = Vec2(0.0f, 0.0f);
		vfixed.push_back(vd);
	}

	// --- Process command list as triangle strips ---
	std::vector<VertData> stack(256);
	int vertexIndex = 0;
	int stripLength = 0;

	VertData temp[4] = {};

	// Buffer for emitted triangles
	struct EmittedTri {
		Vec3 pos[3];
		Color color[3];
		Vec2 uv[3];
		std::string texturePath;
	};
	std::vector<EmittedTri> emittedTriangles;

	for (size_t ci = 0; ci < numCommands; ci++)
	{
		const PSX::InstDrawCommand& cmd = commands[ci];

		if (!cmd.readNextVertFromStackIndexFlag)
		{
			stack[cmd.stackWriteLocationIndex] = vfixed[vertexIndex];
			vertexIndex++;
		}

		// Shift rolling buffer (GodotCTR: temp[0]=temp[1]; temp[1]=temp[2]; temp[2]=temp[3])
		temp[0] = temp[1];
		temp[1] = temp[2];
		temp[2] = temp[3];
		temp[3] = stack[cmd.stackWriteLocationIndex];

		// Assign color from palette
		temp[3].color = Color(static_cast<unsigned char>(128), static_cast<unsigned char>(128), static_cast<unsigned char>(128));
		if (cmd.colorCoordIndex < colors.size())
		{
			temp[3].color = colors[cmd.colorCoordIndex];
		}

		// swapFlag: copy temp[0] over temp[1] (GodotCTR: temp[1] = temp[0])
		if (cmd.swapFlag)
		{
			temp[1] = temp[0];
		}

		// resetFlag: start new strip (GodotCTR: stripLength = 0)
		if (cmd.resetFlag)
		{
			stripLength = 0;
		}

		// Emit triangle when stripLength >= 2 (GodotCTR: for z in range(2,-1,-1): temp[z+1])
		if (stripLength >= 2)
		{
			EmittedTri tri;

			// UVs are assigned from the current command's texture layout at emit time
			Vec2 emitUVs[3] = { Vec2(0, 0), Vec2(0, 0), Vec2(0, 0) };
			int texIdx = cmd.texCoordIndex;
			if (texIdx > 0 && texIdx - 1 < texUVs.size() / 4)
			{
				// GodotCTR: textureLayout["normUV"][z] for z=2,1,0 → uv[2], uv[1], uv[0]
				emitUVs[0] = texUVs[(texIdx - 1) * 4 + 2];
				emitUVs[1] = texUVs[(texIdx - 1) * 4 + 1];
				emitUVs[2] = texUVs[(texIdx - 1) * 4 + 0];
			}

			// Emit: temp[3], temp[2], temp[1] (matching GodotCTR's z+1 with z=2,1,0)
			tri.pos[0] = temp[3].pos;
			tri.pos[1] = temp[2].pos;
			tri.pos[2] = temp[1].pos;
			tri.color[0] = temp[3].color;
			tri.color[1] = temp[2].color;
			tri.color[2] = temp[1].color;
			tri.uv[0] = emitUVs[0];
			tri.uv[1] = emitUVs[1];
			tri.uv[2] = emitUVs[2];

			// Set texture path
			if (texIdx > 0 && texIdx - 1 < texIdxToPath.size())
				tri.texturePath = texIdxToPath[texIdx - 1];

			emittedTriangles.push_back(tri);

			// FlipNormal: swap last 2 vertices of most recent triangle
			if (cmd.normalFlipFlag)
			{
				auto& last = emittedTriangles.back();
				std::swap(last.pos[1], last.pos[2]);
				std::swap(last.color[1], last.color[2]);
				std::swap(last.uv[1], last.uv[2]);
			}
		}

		stripLength++;
	}

	// Convert to Primitive list
	for (const auto& et : emittedTriangles)
	{
		Tri tri;
		tri.p[0].pos = et.pos[0];
		tri.p[1].pos = et.pos[1];
		tri.p[2].pos = et.pos[2];
		tri.p[0].color = et.color[0];
		tri.p[1].color = et.color[1];
		tri.p[2].color = et.color[2];
		tri.p[0].uv = et.uv[0];
		tri.p[1].uv = et.uv[1];
		tri.p[2].uv = et.uv[2];

		tri.texture = et.texturePath;

		Vec3 e1 = tri.p[1].pos - tri.p[0].pos;
		Vec3 e2 = tri.p[2].pos - tri.p[0].pos;
		Vec3 n = e1.Cross(e2);
		if (n.LengthSquared() > 0.0001f) { n.Normalize(); }
		tri.p[0].normal = n;
		tri.p[1].normal = n;
		tri.p[2].normal = n;

		result.push_back(tri);
	}

	return result;
}


void Level::GenerateRenderInstanceData()
{
	Model* instanceModel = m_models[LevelModels::INSTANCES];
	if (!instanceModel) { return; }

	instanceModel->ClearModels();

	if (m_instances.empty())
	{
		instanceModel->GetMesh().Clear();
		return;
	}

	constexpr float labelHeightOffset = 3.0f;

	for (size_t i = 0; i < m_instances.size(); i++)
	{
		const Instance& inst = m_instances[i];
		const Vec3& pos = inst.GetPos();
		const std::string& modelName = inst.GetModelName();

		// Geometry child (always created, ensures stride = 2 per instance)
		Model* childModel = instanceModel->AddModel();
		if (!modelName.empty())
		{
			auto cacheIt = m_parsedModelCache.find(modelName);
			if (cacheIt == m_parsedModelCache.end())
			{
				auto importIt = m_importedModels.find(modelName);
				if (importIt != m_importedModels.end())
				{
					std::string texCacheDir = (std::filesystem::temp_directory_path() / "CTE_tex_cache").string();
					std::vector<Primitive> primitives = ParseCtrModelGeometry(importIt->second, texCacheDir, modelName);
					if (!primitives.empty())
					{
						m_parsedModelCache[modelName] = primitives;
						cacheIt = m_parsedModelCache.find(modelName);
					}
				}
			}

			if (cacheIt != m_parsedModelCache.end() && !cacheIt->second.empty())
			{
				childModel->GetMesh().SetGeometry(
					cacheIt->second,
					Mesh::RenderFlags::DrawBackfaces | Mesh::RenderFlags::DontOverrideRenderFlags
				);
			}
		}
		childModel->SetPosition(inst.GetPos());
		childModel->SetRotation(inst.GetRot());
		childModel->SetScale(inst.GetScale());

		// Label with instance name
		Model* label = instanceModel->AddModel();
		std::string labelText = inst.GetName();
		if (labelText.empty())
			labelText = "Instance " + std::to_string(i + 1);
		label->GetMesh().SetGeometry(labelText, Text3D::Align::CENTER, Color(static_cast<unsigned char>(0), static_cast<unsigned char>(200), static_cast<unsigned char>(200), static_cast<unsigned char>(255)));
		Vec3 labelPos = pos;
		labelPos.y += labelHeightOffset;
		label->SetPosition(labelPos);
	}

	// Placeholder mesh so parent Model::IsReady() returns true
	std::vector<Primitive> dummy;
	dummy.push_back(Tri(Point(0,0,0,0,0,0), Point(0,0,0,0,0,0), Point(0,0,0,0,0,0)));
	instanceModel->GetMesh().SetGeometry(
		dummy,
		Mesh::RenderFlags::DrawBackfaces | Mesh::RenderFlags::DontOverrideRenderFlags
	);
}


void Level::GenerateRenderStartpointData()
{
	if (!m_models[LevelModels::SPAWN]) { return; }

	std::vector<Primitive> spawnsTriangles;
	spawnsTriangles.reserve(m_spawn.size() * 8);

	for (const Spawn& e : m_spawn)
	{
		Vertex v = Vertex(Point(e.pos.x, e.pos.y, e.pos.z, 0, 128, 255));
		const std::vector<Primitive> tris = v.ToGeometry();
		spawnsTriangles.insert(spawnsTriangles.end(), tris.begin(), tris.end());
	}

	m_models[LevelModels::SPAWN]->GetMesh().SetGeometry(spawnsTriangles, Mesh::RenderFlags::DrawBackfaces | Mesh::RenderFlags::DontOverrideRenderFlags);
}

void Level::GenerateRenderSkyboxData()
{
	if (!m_models[LevelModels::SKYBOX]) { return; }

	std::vector<Primitive> triangles = m_skybox.ToGeometry(m_bsp.GetBoundingBox());
	m_models[LevelModels::SKYBOX]->GetMesh().SetGeometry(triangles, Mesh::RenderFlags::DrawBackfaces | Mesh::RenderFlags::DontOverrideRenderFlags);
}

void Level::GenerateRenderSelectedBlockData(const Quadblock& quadblock, const Vec3& queryPoint)
{
	if (!m_models[LevelModels::SELECTED]) { return; }

	m_rendererQueryPoint = queryPoint;

	std::vector<Primitive> triangles;
	triangles.reserve(m_rendererSelectedQuadblockIndexes.size() * 8 + 8);

	const std::filesystem::path emptyTexturePath;
	const std::array<QuadUV, NUM_FACES_QUADBLOCK + 1> emptyUvs = {};
	for (size_t index : m_rendererSelectedQuadblockIndexes)
	{
		const Quadblock& qb = m_quadblocks[index];
		std::vector<Primitive> qbTriangles = qb.ToGeometry(false, &emptyUvs, &emptyTexturePath);
		for (Primitive& primitive : qbTriangles)
		{
			for (unsigned i = 0; i < primitive.pointCount; i++) { primitive.p[i].color = primitive.p[i].color.Negated(); }
			triangles.push_back(primitive);
		}
	}

	Vertex v = Vertex(Point(queryPoint.x, queryPoint.y, queryPoint.z, 255, 0, 0));
	const std::vector<Primitive> queryTriangles = v.ToGeometry();
	triangles.insert(triangles.end(), queryTriangles.begin(), queryTriangles.end());

	m_models[LevelModels::SELECTED]->GetMesh().SetGeometry(triangles,
		Mesh::RenderFlags::DrawWireframe | Mesh::RenderFlags::DrawBackfaces | Mesh::RenderFlags::ForceDrawOnTop | Mesh::RenderFlags::DrawLinesAA | Mesh::RenderFlags::DontOverrideRenderFlags | Mesh::RenderFlags::QuadblockLod,
		Mesh::ShaderFlags::Blinky);

	if (GuiRenderSettings::showVisTree)
	{
		std::vector<const BSP*> bspLeaves = m_bsp.GetLeaves();
		size_t myBSPIndex = 0;
		for (size_t bsp_index = 0; bsp_index < bspLeaves.size(); bsp_index++)
		{
			const BSP& bsp = *bspLeaves[bsp_index];
			if (bsp.GetId() == quadblock.GetBSPID()) { myBSPIndex = bsp_index; }
		}

		std::vector<Primitive> multiTriangles;
		for (size_t bsp_index = 0; bsp_index < bspLeaves.size(); bsp_index++)
		{
			const BSP& bsp = *bspLeaves[bsp_index];
			if (m_bspVis.Get(myBSPIndex, bsp_index))
			{
				const std::vector<size_t> qbIndeces = bsp.GetQuadblockIndexes();
				for (size_t qbInd : qbIndeces)
				{
					Quadblock& qb = m_quadblocks[qbInd];
					std::vector<Primitive> qbTriangles = qb.ToGeometry(false, &emptyUvs, &emptyTexturePath);
					for (Primitive& primitive : qbTriangles)
					{
						for (unsigned i = 0; i < primitive.pointCount; i++) { primitive.p[i].color = primitive.p[i].color.Negated(); }
						multiTriangles.push_back(primitive);
					}
				}
			}
		}

		m_models[LevelModels::MULTI_SELECTED]->GetMesh().SetGeometry(multiTriangles,
			Mesh::RenderFlags::DrawWireframe | Mesh::RenderFlags::DrawBackfaces | Mesh::RenderFlags::ForceDrawOnTop | Mesh::RenderFlags::DrawLinesAA | Mesh::RenderFlags::DontOverrideRenderFlags | Mesh::RenderFlags::QuadblockLod,
			Mesh::ShaderFlags::Blinky);
	}
}

void Level::ViewportClickHandleBlockSelection(int pixelX, int pixelY, bool appendSelection, const Renderer& rend)
{
	std::function<std::optional<std::tuple<const Quadblock*, const glm::vec3>>(int, int, std::vector<Quadblock>&, unsigned)> check = [&rend](int pixelCoordX, int pixelCoordY, const std::vector<Quadblock>& qbs, unsigned index)
		{
			std::vector<std::tuple<const Quadblock*, glm::vec3, float>> passed;

			for (const Quadblock& qb : qbs)
			{
				bool collided = false;
				const Vertex* verts = qb.GetUnswizzledVertices();
				glm::vec3 tri[3];
				bool isQuadblock = qb.IsQuadblock();

				std::tuple<glm::vec3, float> queryResult;
				glm::vec3 worldSpaceRay = rend.ScreenspaceToWorldRay(pixelCoordX, pixelCoordY);

				tri[0] = glm::vec3(verts[0].m_pos.x, verts[0].m_pos.y, verts[0].m_pos.z);
				tri[1] = glm::vec3(verts[2].m_pos.x, verts[2].m_pos.y, verts[2].m_pos.z);
				tri[2] = glm::vec3(verts[6].m_pos.x, verts[6].m_pos.y, verts[6].m_pos.z);

				queryResult = rend.WorldspaceRayTriIntersection(worldSpaceRay, tri);
				collided |= (std::get<1>(queryResult) != -1.0f);

				if (collided) { passed.push_back(std::tuple<const Quadblock*, glm::vec3, float>(&qb, std::get<0>(queryResult), std::get<1>(queryResult))); continue; }

				if (!isQuadblock) { continue; }

				tri[0] = glm::vec3(verts[2].m_pos.x, verts[2].m_pos.y, verts[2].m_pos.z);
				tri[1] = glm::vec3(verts[6].m_pos.x, verts[6].m_pos.y, verts[6].m_pos.z);
				tri[2] = glm::vec3(verts[8].m_pos.x, verts[8].m_pos.y, verts[8].m_pos.z);

				queryResult = rend.WorldspaceRayTriIntersection(worldSpaceRay, tri);
				collided |= (std::get<1>(queryResult) != -1.0f);

				if (collided) { passed.push_back(std::tuple<const Quadblock*, glm::vec3, float>(&qb, std::get<0>(queryResult), std::get<1>(queryResult))); continue; }
			}

			// sort collided blocks by time value (distance from camera).
			std::sort(passed.begin(), passed.end(),
				[](const std::tuple<const Quadblock*, glm::vec3, float>& a, const std::tuple<const Quadblock*, glm::vec3, float>& b) {
					return std::get<2>(a) < std::get<2>(b);
				});

			std::optional<std::tuple<const Quadblock*, glm::vec3>> result;
			if (passed.size() > 0)
			{
				const auto& tuple = passed[index % passed.size()];
				const Quadblock* qb = std::get<0>(tuple);
				result = std::make_optional(std::tuple<const Quadblock*, glm::vec3>(qb, std::get<1>(tuple)));
			}
			else { result.reset(); }
			return result;
		};

	static int lastClickedX = pixelX;
	static int lastClickedY = pixelY;
	static int indenticalClickTimes = -1;

	if (!appendSelection && pixelX == lastClickedX && pixelY == lastClickedY)
	{
		indenticalClickTimes++;
	}
	else
	{
		lastClickedX = pixelX;
		lastClickedY = pixelY;
		indenticalClickTimes = 0;
	}

	std::optional<std::tuple<const Quadblock*, const glm::vec3>> collidedQB = check(pixelX, pixelY, m_quadblocks, indenticalClickTimes);

	if (collidedQB.has_value())
	{
		const Quadblock* clickedQuadblock = std::get<0>(collidedQB.value());
		glm::vec3 p = std::get<1>(collidedQB.value());
		Vec3 point = Vec3(p.x, p.y, p.z);
		size_t clickedIndex = REND_NO_SELECTED_QUADBLOCK;
		for (size_t i = 0; i < m_quadblocks.size(); i++)
		{
			if (&m_quadblocks[i] == clickedQuadblock)
			{
				clickedIndex = i;
				break;
			}
		}

		if (clickedIndex != REND_NO_SELECTED_QUADBLOCK)
		{
			if (!appendSelection) { m_rendererSelectedQuadblockIndexes.clear(); }

			auto selectedIt = std::find(m_rendererSelectedQuadblockIndexes.begin(), m_rendererSelectedQuadblockIndexes.end(), clickedIndex);
			bool alreadySelected = selectedIt != m_rendererSelectedQuadblockIndexes.end();
			if (appendSelection && alreadySelected)
			{
				m_rendererSelectedQuadblockIndexes.erase(selectedIt);
			}
			else if (!alreadySelected)
			{
				m_rendererSelectedQuadblockIndexes.push_back(clickedIndex);
			}
		}

		GenerateRenderSelectedBlockData(*clickedQuadblock, point);
	}
	else
	{
		m_models[LevelModels::SELECTED]->GetMesh().Clear();
	}
	UpdateRenderCheckpointData();
}
