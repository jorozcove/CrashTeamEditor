#pragma once

#include "quadblock.h"

#include <unordered_map>
#include <unordered_set>
#include <string>

enum class MaterialType
{
	TERRAIN, QUAD_FLAGS, DRAW_FLAGS, CHECKPOINT, TEXTURE_INDEX
};

class MaterialBase
{
public:
	virtual void Restore() = 0;
	virtual void Clear() = 0;
};

template <typename T, MaterialType M>
class MaterialProperty : public MaterialBase
{
public:
	MaterialProperty();
	void SetPreview(const std::string& material, const T& preview);
	void SetDefaultValue(const std::string& material, const T& value);
	bool UnsavedChanges(const std::string& material) const;
	void Restore() override;
	void Clear() override;
	T& GetPreview(const std::string& material);
	const T& GetBackup(const std::string& material);
	void Apply(const std::string& material, const std::vector<size_t>& quadblockIndexes, std::vector<Quadblock>& quadblocks);

private:
	std::unordered_map<std::string, T> m_backup;
	std::unordered_map<std::string, T> m_preview;
	std::unordered_set<std::string> m_materialsChanged;
};

void ClearMaterials();
void RestoreMaterials();