// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// A very simple example that can be used as template project for
// a Chrono::Engine simulator with 3D view.
// =============================================================================

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "StlWriter.h"
#include "Obj3d.h"
// Use the namespace of Chrono

using namespace chrono;
using namespace chrono::irrlicht;
// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;
//initialize functions
void create3dPrinterContainer(ChIrrApp& application, std::shared_ptr<ChSystemNSC> mphysicalSystem);
std::shared_ptr <ChMaterialSurfaceNSC> defineObjMatProps();
void dropInContainer(ChIrrApp& app, std::shared_ptr<ChSystemNSC> mphysicalSystem, std::vector<std::shared_ptr<Obj>>& myObjVec, double timeForDisp);
//initialize vars
std::vector<std::shared_ptr<Obj>> objVec; //vector for 3d printed object

//================================================================
//edit these sim variables
double dT = 0.005;// [s/step]
double g = 9.81; //gravity [m/s^2]
//Set the following diminsions to your liking these will be the interior diminsions of the box
double boxW = .10; // longest diminsion will be the perpendicular to view  [m]
double boxL = .05;//[m]
double boxH = .1;//box height [m]
double thick = .01; //thickness of walls [m]
double tiltAngle = 45; //tilt angle of guiding walls above box [degrees]

double density = 1070; //density of abs plastic pulled from wolfram alpha [kg/m^3]
double numPerLayer = 10;
double numLayers = 5;

std::string objectFname = "tractor_wheel_fine.obj"; //object filename
std::string textureName = "rock.jpg";				//texture filename
//================================================================




//set properties for crated objects here
std::shared_ptr <ChMaterialSurfaceNSC> defineObjMatProps() 
{
	auto mat = std::make_shared<ChMaterialSurfaceNSC>();
	//double sfric = 0.4; //static friction
	//double kfric = 0.3; //kinetic friction
	double fric = 0; //non-zero value makes sim 2x slower sets both kinetic and static to same value
	double compliance = 0; //normal direction compliance, It is the inverse of the stiffness [K], measured in m/N, perfect rigid contact = 0
	double complianceT = 0;  //Compliance of the contact, in tangential direction. m/N, perfect rigid contact = 0
	double restitution = 0;  //The normal restitution coefficient, for collisions, value between 0-1.

	//mat->static_friction = sfric;
	//mat->SetKfriction = kfric;
	mat->SetFriction(fric);
	mat->SetCompliance(compliance);
	mat->SetComplianceT(complianceT);
	mat->SetRestitution(restitution);
	return mat;
}
//drops particles 
void dropInContainer(ChIrrApp& app, std::shared_ptr<ChSystemNSC> mphysicalSystem, std::vector<std::shared_ptr<Obj>> & myObjVec, double timeForDisp)
{
	std::shared_ptr<Obj>obj0 = std::make_shared<Obj>(mphysicalSystem);
	auto mat = defineObjMatProps(); //define material properties
	//==========================================
	//compute position and rotations for initial drop position of each object
	ChVector<> pos = ChVector<>(0, 0, 0);
	ChQuaternion<> rot = QUNIT;
	ChVector<> dropSpeed = ChVector<>(0, g * timeForDisp / 2.0,0);
	///TODO write code which creates initial positions for particles

	//offset height of objects to at least 1.5*height of box
	pos = pos + ChVector<>(0, 1.5 * boxH, 0);
	//==========================================
	//apply computed values to system and create object

	obj0->Properties(mat, density, pos, rot, objectFname, textureName);
	obj0->Create();


	//must set initial speed after obj is created
	obj0->body->SetPos_dt(dropSpeed);

	objVec.emplace_back((std::shared_ptr<Obj>)obj0);

	app.AssetBindAll();
	app.AssetUpdateAll();
}
int main(int argc, char* argv[]) {
    // Set path to Chrono data directory
    SetChronoDataPath(PROJECT_DATA_DIR);
    
    // Create a Chrono physical system
	std::shared_ptr<ChSystemNSC>  mphysicalSystem = std::make_shared<ChSystemNSC>();
	
    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(mphysicalSystem.get(), L"design 3d printer stl", core::dimension2d<u32>(800, 600),
                         false);  // screen dimensions

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
	
	RTSCamera* camera = new RTSCamera(application.GetDevice(), application.GetDevice()->getSceneManager()->getRootSceneNode(),
	application.GetDevice()->getSceneManager(), -1.0f, -50.0f, 0.5f, 0.0005f);
	camera->setTranslateSpeed(0.005f);
	camera->setPosition(core::vector3df(0, float(0.8*boxH),float(-4.0*boxL)));
	camera->setTarget(core::vector3df(0, float(0.3*boxH), 0));
	camera->setNearValue(0.005f);


	create3dPrinterContainer(application, mphysicalSystem);

	//===============================================
	//use below code to test locations

	//double sRad = boxL/2.0;
	//auto  sphere = std::make_shared<ChBodyEasySphere>(sRad,
	//	1000,          // density
	//	true,          // collide enable?
	//	true);         // visualization?
	//sphere->SetPos(ChVector<>(0, sRad, 0));
	//mphysicalSystem.Add(sphere);
	//sphere->SetBodyFixed(true);


	//auto pallet = std::make_shared<ChBodyEasyMesh>(
	//	GetChronoDataFile("pallet.obj"),  // mesh .OBJ file
	//	300,                              // density
	//	true,   // compute mass, inertia & COG from the mesh (must be a closed watertight mesh!)
	//	true,   // enable collision with mesh
	//	0.001,  // sphere swept inflate of mesh - improves robustness of collision detection
	//	true);  // enable visualization of mesh
	//application.GetSystem()->Add(pallet);
	//pallet->SetPos(ChVector<>(0, 3, 0));

	//// apply also a texture to the pallet:
	//auto pallet_texture = std::make_shared<ChTexture>();
	//pallet_texture->SetTextureFilename(GetChronoDataFile("cubetexture.png"));
	//pallet->AddAsset(pallet_texture);
    //======================================================================


    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    // Adjust some settings:
    application.SetTimestep(dT);
    application.SetTryRealtime(true);

    //
    // THE SOFT-REAL-TIME CYCLE
    //

	double t = mphysicalSystem->GetChTime();
	double timeForVerticalDisplacement = 0.015; //time in s to wait between deposits if dep number is > 1

	application.DrawAll();
    while (application.GetDevice()->run()) {
		t = mphysicalSystem->GetChTime();
		if ((fmod(t, timeForVerticalDisplacement) < dT) && (objVec.size() < numPerLayer * numLayers))
			dropInContainer(application,mphysicalSystem, objVec, timeForVerticalDisplacement);

        application.BeginScene();

		//if you will be updating the assets based on per-timestep calculations uncomment the below
		//for (size_t i = 0; i < objVec.size(); ++i)
		//{
		//	application.AssetUpdate(objVec[i]->body);
		//}

		

        application.DrawAll();

        // This performs the integration timestep!
        application.DoStep();

        application.EndScene();
    }

    return 0;
}


//===========================================================================================================
//this function creates the open box and bottom floor
void create3dPrinterContainer(ChIrrApp& application, std::shared_ptr<ChSystemNSC> mphysicalSystem)
{
	auto floorBody = std::make_shared<ChBodyEasyBox>(10, 2, 10,  // x, y, z dimensions
		3000,       // density
		false,      // no contact geometry
		true        // enable visualization geometry
		);
	floorBody->SetPos(ChVector<>(0, -2, 0));
	floorBody->SetBodyFixed(true);
	//attach a RGB color asset to the floor, for better visualization
	auto color = std::make_shared<ChColorAsset>();
	color->SetColor(ChColor(0.2f, 0.45f, 0.25f));
	floorBody->AddAsset(color);
	mphysicalSystem->Add(floorBody);


	ChVector<> boxDim = (boxW, boxH, boxL);
	////////////////////////////////////////////////////////////

	//(0,0,0) is the central point in the box, where the Y position is the on top of the bottom surface

	//you will be looking at front wall
	auto frontWall = std::make_shared<ChBodyEasyBox>(boxW, boxH + thick, thick,  // x,y,z size
		1000,          // density
		true,          // collide enable?
		false);         // visualization?
	//backWall is farthest away
	auto  backWall = std::make_shared<ChBodyEasyBox>(boxW, boxH + thick, thick,
		1000,          // density
		true,          // collide enable?
		true);         // visualization?
	auto  leftWall = std::make_shared<ChBodyEasyBox>(thick, boxH + thick, boxL + 2 * thick,
		1000,          // density
		true,          // collide enable?
		true);         // visualization?
	auto  rightWall = std::make_shared<ChBodyEasyBox>(thick, boxH + thick, boxL + 2 * thick,
		1000,          // density
		true,          // collide enable?
		true);         // visualization?	
	auto  bottomWall = std::make_shared<ChBodyEasyBox>(boxW, thick, boxL + 2 * thick,
		1000,          // density
		true,          // collide enable?
		true);         // visualization?

	//--- The following are angled walls above the box which guide the generated printed parts into the box
	auto leftTiltWall = std::make_shared<ChBodyEasyBox>(thick, boxL, boxL + 2 * thick,
		1000,          // density
		true,          // collide enable?
		true);         // visualization?
	auto rightTiltWall = std::make_shared<ChBodyEasyBox>(thick, boxL, boxL + 2 * thick,
		1000,          // density
		true,          // collide enable?
		true);         // visualization?
	auto backTiltWall = std::make_shared<ChBodyEasyBox>(boxW + 2 * thick, boxL, thick,
		1000,          // density
		true,          // collide enable?
		true);         // visualization?


	frontWall->SetPos(ChVector<>(0, (boxH + thick) / 2.0 - thick, -(boxL + thick) / 2.0));
	backWall->SetPos(ChVector<>(frontWall->GetPos().x(), frontWall->GetPos().y(), -1 * frontWall->GetPos().z()));
	leftWall->SetPos(ChVector<>(-(boxW + thick) / 2.0, frontWall->GetPos().y(), 0));
	rightWall->SetPos(ChVector<>(-leftWall->GetPos().x(), frontWall->GetPos().y(), 0));
	bottomWall->SetPos(ChVector<>(0, -thick / 2.0, 0));

	tiltAngle = tiltAngle * CH_C_PI / 180.0;

	ChQuaternion<> leftRotation = Angle_to_Quat(AngleSet::RXYZ, ChVector<>(0, 0, -tiltAngle));
	ChQuaternion<> rightRotation = Angle_to_Quat(AngleSet::RXYZ, ChVector<>(0, 0, tiltAngle));
	ChQuaternion<> backRotation = Angle_to_Quat(AngleSet::RXYZ, ChVector<>(-tiltAngle, 0, 0));

	leftTiltWall->SetPos(ChVector<>(-(boxW + (boxL + thick) * cos(tiltAngle)) / 2.0, boxH + boxL / 2.0 - ((boxL - 2 * thick) * sin(tiltAngle) / 2.0), 0));
	rightTiltWall->SetPos(ChVector<>(-1 * leftTiltWall->GetPos().x(), leftTiltWall->GetPos().y(), 0));
	backTiltWall->SetPos(ChVector<>(0, leftTiltWall->GetPos().y(), (boxL + (boxL + thick) * cos(tiltAngle)) / 2.0));

	leftTiltWall->SetRot(leftRotation);
	rightTiltWall->SetRot(rightRotation);
	backTiltWall->SetRot(backRotation);
	//---------- this code removes keeps me from having to repeat many lines of code
	int wallFam = 2;
	auto mtexturewall = std::make_shared<ChTexture>();
	auto mtexturewall2 = std::make_shared<ChTexture>();

	mtexturewall->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
	mtexturewall2->SetTextureFilename(GetChronoDataFile("pink.png"));

	std::vector<std::shared_ptr<ChBodyEasyBox>> wallVec = { frontWall,backWall,leftWall,rightWall,bottomWall,leftTiltWall,rightTiltWall,backTiltWall };
	for (size_t i = 0; i < wallVec.size(); i++)
	{
		std::shared_ptr<ChBodyEasyBox> wallPtr = wallVec[i];
		wallPtr->SetBodyFixed(true);
		mphysicalSystem->Add(wallPtr);
		wallPtr->GetCollisionModel()->SetFamily(wallFam);
		wallPtr->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(wallFam);
		if (i <= 3)
			wallPtr->AddAsset(mtexturewall);
		else
			wallPtr->AddAsset(mtexturewall2);
	}


}


