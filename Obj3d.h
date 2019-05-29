#ifndef OBJ3D_H_
#define OBJ3D_H_

#include <chrono>
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include <iostream>
#include "assets/ChTexture.h"
#include "chrono_irrlicht/ChIrrApp.h"//changed path from unit to chrono to reflect changes in updated chrono
#include "chrono_irrlicht/ChIrrTools.h"
#include <irrlicht.h>
//#include "physics/ChSystem.h"  // Arman: take care of this later
//#include "chrono_parallel/physics/ChSystemParallel.h"
#include <memory>
#include <deque>
//#include "common.h"


namespace chrono {
	class Obj {
	public:
		Obj();
		Obj(std::shared_ptr<ChSystemNSC> otherSystem, int id = -1);
		Obj(std::shared_ptr<Obj> temp);
		~Obj();
		virtual void SetFilename(std::string fname);
		virtual void SetId(int id);
		virtual int GetId();
		virtual void Create();

		virtual void Properties(
			std::shared_ptr<ChMaterialSurface> surfaceMaterial,
			double other_density= 1070, //density of abs plastic in [kg/m^3] pulled from wolfram alpha,
			ChVector<> other_pos = VNULL,
			ChQuaternion<> other_rot = QUNIT,
			std::string obj_fname = "",
			std::string texture_fname="blu.png"
			);


		std::string m_texture_fname;
		std::shared_ptr<ChMaterialSurface> m_mat;
		ChVector<> pos;
		ChQuaternion<> rot;
		double density;
		std::shared_ptr<ChTexture> m_texture;


		std::shared_ptr<ChBodyEasyMesh> body;
	private:
		int m_id;
		std::string m_fname;
		std::shared_ptr<ChSystemNSC> m_system;
	};
};
#endif /* OBJ3D_H_ */