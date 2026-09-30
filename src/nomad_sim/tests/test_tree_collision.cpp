// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, NOMAD contributors
// Standalone CPU-only regression check. No ROS, ZMQ connection, GUI or renderer.
// Scene membership verifies the input to render-based sensors, not pixel output.

#include <box2d/box2d.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mvsim/Block.h>
#include <mvsim/World.h>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
void require(bool condition, const std::string& message)
{
	if (!condition)
	{
		throw std::runtime_error(message);
	}
}

std::string xmlEscape(const std::string& input)
{
	std::string result;
	for (const char c : input)
	{
		switch (c)
		{
			case '&':
				result += "&amp;";
				break;
			case '<':
				result += "&lt;";
				break;
			case '>':
				result += "&gt;";
				break;
			default:
				result += c;
				break;
		}
	}
	return result;
}

mvsim::Block::Ptr makeTree(
	mvsim::World& world, const std::string& model, const std::string& name, bool intangible)
{
	// Leave the near tree's intangible tag absent: test the default collider.
	std::string xml = "<block name=\"" + name + "\">"
					  "<static>true</static><zmin>0</zmin><zmax>4.5</zmax>"
					  "<init_pose3d>0 0 0 0 0 0</init_pose3d><skip_elevation_adjust/>"
					  "<shape><pt>-0.22 -0.22</pt><pt>0.22 -0.22</pt>"
					  "<pt>0.22 0.22</pt><pt>-0.22 0.22</pt></shape>"
					  "<visual><name>" + name + "_mesh</name><model_uri>" + xmlEscape(model) +
					  "</model_uri></visual>";
	if (intangible)
	{
		xml += "<intangible>true</intangible>";
	}
	xml += "</block>";
	return mvsim::Block::factory(&world, xml);
}
}  // namespace

int main(int argc, char** argv)
{
	if (argc != 2)
	{
		std::cerr << "Usage: test_tree_collision /absolute/path/to/pine_tree.obj\n";
		return 2;
	}
	try
	{
		const auto model = std::filesystem::absolute(argv[1]);
		require(std::filesystem::is_regular_file(model), "Missing local pine_tree.obj");
		mvsim::World world;
		world.headless(true);
		world.internal_initialize();
		const int baselineBodies = world.getBox2DWorld()->GetBodyCount();
		const int baselineJoints = world.getBox2DWorld()->GetJointCount();

		auto farTree = makeTree(world, model.string(), "far_intangible_tree", true);
		require(farTree->getBox2DBlockBody() == nullptr, "Intangible tree created a Box2D body");
		require(farTree->isStatic(), "Intangible tree is not static");
		require(
			world.getBox2DWorld()->GetBodyCount() == baselineBodies,
			"Intangible tree increased the Box2D body count");
		require(
			world.getBox2DWorld()->GetJointCount() == baselineJoints,
			"Intangible tree increased the Box2D joint count");
		// Regression assertion for the local intangible-height fix. The old
		// upstream library fails this intentionally: a visual-only tree must
		// not raise a vehicle to the top of its trunk/canopy.
		require(
			!farTree->getElevationAt({0.0, 0.0}).has_value(),
			"Intangible tree still contributes to physical ground elevation");

		auto nearTree = makeTree(world, model.string(), "near_default_tree", false);
		require(nearTree->getBox2DBlockBody() != nullptr, "Default tree lost its Box2D collider");
		require(
			nearTree->getBox2DBlockBody()->GetType() == b2_staticBody,
			"Default near tree is not a static Box2D body");
		require(
			nearTree->getBox2DBlockBody()->GetFixtureList() != nullptr,
			"Default near tree lost its collision fixture");
		require(
			world.getBox2DWorld()->GetBodyCount() == baselineBodies + 1,
			"Default tree did not add exactly one Box2D body");
		const auto nearHeight = nearTree->getElevationAt({0.0, 0.0});
		require(
			nearHeight.has_value() && std::abs(nearHeight.value() - 4.5f) < 1e-6f,
			"Default near tree lost its 4.5m physical top elevation");

		mrpt::opengl::COpenGLScene visual;
		mrpt::opengl::COpenGLScene physical;
		farTree->guiUpdate(visual, physical);
		nearTree->guiUpdate(visual, physical);
		for (const auto* name : {"far_intangible_tree_mesh", "near_default_tree_mesh"})
		{
			require(
				static_cast<bool>(visual.getByName(name)),
				std::string(name) + " absent from visual scene");
			require(
				static_cast<bool>(physical.getByName(name)),
				std::string(name) + " absent from render-based sensor physical scene");
		}

		std::cout << "PASS: intangible far tree adds no Box2D body or joint; "
					 "default near tree retains its static collision fixture.\n"
				  << "PASS: intangible tree contributes no physical elevation; "
					 "default near tree retains its 4.5m elevation.\n"
				  << "PASS: both actual pine OBJ meshes remain in visual and sensor physical scenes.\n"
				  << "Box2D body count: baseline=" << baselineBodies
				  << ", far=" << baselineBodies
				  << ", far+near=" << world.getBox2DWorld()->GetBodyCount() << "\n"
				  << "Box2D joint count: baseline=" << baselineJoints
				  << ", far=" << baselineJoints
				  << ", far+near=" << world.getBox2DWorld()->GetJointCount() << "\n";
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "FAIL: " << e.what() << "\n";
		return 1;
	}
}
