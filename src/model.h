#pragma once

#include "mesh.h"
#include "text3d.h"
#include "transform.h"

#include <functional>
#include <list>
#include <string>

class Model : public Transform
{
public:
	Model();
	~Model();
  Mesh& GetMesh();
	void SetRenderCondition(const std::function<bool()>& renderCondition);
	Model* AddModel();
	void ClearModels();
	bool RemoveModel(Model* model);
	void Clear(bool models);
	bool IsReady() const;
	size_t GetModelCount() const { return m_child.size(); }
	Model* GetModel(size_t index) const {
		if (index >= m_child.size()) return nullptr;
		auto it = m_child.begin();
		std::advance(it, index);
		return *it;
	}

private:
	Mesh m_mesh;
	std::function<bool()> m_renderCondition;
	std::list<Model*> m_child;

	friend class Renderer;
};
