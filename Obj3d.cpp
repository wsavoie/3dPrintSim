#include "Obj3d.h"

using namespace chrono;


Obj::Obj()
{
	Obj(NULL, -1);
}
Obj::Obj(std::shared_ptr<ChSystemNSC> otherSystem, int id) : m_system(otherSystem), m_id(id)
{
	body = std::make_shared<ChBody>();
	m_texture = std::make_shared<ChTexture>();
}
Obj::Obj(std::shared_ptr<Obj> temp)
{
	body = std::make_shared<ChBody>();
	m_system=temp->m_system;
	m_id = temp->GetId();
	m_fname = temp->m_fname;
	this->Properties(temp->m_mat, temp->density, temp->pos, temp->rot, temp->m_fname, temp->m_texture_fname);
}
void Obj::Properties(
	std::shared_ptr<ChMaterialSurface> surfaceMaterial,
	double other_density,
	ChVector<> other_pos,
	ChQuaternion<> other_rot,
	std::string obj_fname,
	std::string texture_fname)
{
	m_mat = surfaceMaterial;
	density = other_density;
	pos = other_pos;
	rot = other_rot;
	m_fname = obj_fname;
	m_texture_fname=texture_fname;

	m_texture->SetTextureFilename(GetChronoDataFile(m_texture_fname));
}

void Obj::SetId(int id)
{
	m_id = id;
}
int Obj::GetId()
{
	return m_id;
}
void Obj::SetFilename(std::string fname)
{
	m_fname = fname;
}
void Obj::Create()
{
	body->SetMaterialSurface(m_mat);
	this->body = std::make_shared<ChBodyEasyMesh>(
		GetChronoDataFile(m_fname),  // mesh .OBJ file
		density,                              // density
		true,   // compute mass, inertia & COG from the mesh (must be a closed watertight mesh!)
		true,   // enable collision with mesh
		0.001,  // sphere swept inflate of mesh - improves robustness of collision detection
		true);  // enable visualization of mesh
	body->SetPos(pos);
	body->SetRot(rot);
	body->SetBodyFixed(false);
	body->GetPhysicsItem()->SetIdentifier(m_id);
	body->AddAsset(m_texture);
	m_system->AddBody(body);
}

Obj::~Obj()
{
	m_system->Remove(body);
}