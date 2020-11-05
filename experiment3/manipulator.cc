#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <nos/print.h>
#include <nos/fprint.h>
#include <nos/terminal.h>

#include <linalg/linalg.h>
#include <ralgo/linalg/backpack.h>
#include <rabbit/space/htrans2.h>

#include <igris/math.h>

extern rabbit::screw2<double> CARGO_TARGET_VELOCITY;
extern rabbit::htrans2<double> CARGO_POSITION;

struct Regulator
{
	bool speed2_loop_enabled = true;
	bool position_loop_enabled = true;

	double speed_error;
	double position_error;

	double speed_target;
	//double position_target = 45. / 180. * 3.14;
	double position_target;

	double speed_integral;
	double position_integral;

	double speed2_target;

	static constexpr double spd_T = 0.03;
	static constexpr double spd_ksi = 0.75;
	static constexpr double spd_A = 9;

	static constexpr double pos_T = 0.15;
	static constexpr double pos_ksi = 2;

	double pos_kp = 2.*pos_ksi/pos_T;
	double pos_ki = 1./pos_T/pos_T;

	double spd_kp = 2.*spd_ksi/spd_T*spd_A;
	double spd_ki = 1./spd_T/spd_T*spd_A;

	//double force_compensation = 0.1;

	double control_signal = 0;

	void reset()
	{
		speed_error = 0;
		position_error = 0;

		speed_target = 0.2;
		position_target = 0;

		speed_integral = 0;
		position_integral = 0;

		speed2_target = 0.1;
	}
};

namespace gazebo
{
	extern physics::WorldPtr WORLD;

	class ModelPush : public ModelPlugin
	{
	private:
		double ForceKoeff = 0.001;
		// Pointer to the model
		physics::ModelPtr model;

		// Pointer to the update event connection
		event::ConnectionPtr updateConnection;

		int inited = 0;
		Regulator joint_base_regulator;
		Regulator joint0_regulator;
		Regulator joint1_regulator;

		double lasttime;
		double starttime;
		double delta;

	public:

		double evaltime() 
		{
			auto t = WORLD->SimTime();

			return (double)t.sec + (double)t.nsec / 1000000000.;
		}

		void Reset()
		{
			joint0_regulator.reset();
			joint1_regulator.reset();

			lasttime = evaltime();

			starttime = evaltime();

			if (model->GetName() == "manip1")
			{
				auto joint0 = model->GetJoint("joint0");
				auto joint1 = model->GetJoint("joint1");
				joint0->SetPosition(0, -3.14 / 4);
				joint1->SetPosition(0, 3.14 / 2);
				joint0_regulator.position_target = -3.14 / 4;
				joint1_regulator.position_target = 3.14 / 2;
			}
			else
			{
				auto joint0 = model->GetJoint("joint0");
				auto joint1 = model->GetJoint("joint1");
				joint0->SetPosition(0, 3.14 / 4);
				joint1->SetPosition(0, -3.14 / 2);
				joint0_regulator.position_target = 3.14 / 4;
				joint1_regulator.position_target = -3.14 / 2;
			}

			inited = 0;
		}

		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
		{
			nos::println("Load manipulator");
			// Store the pointer to the model
			this->model = _parent;

			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			                             std::bind(&ModelPush::OnUpdate, this));

			Reset();
		}

		// Called by the world update start event
	public:

		linalg::vec<double,2> position_integral {};
		void OnUpdate()
		{
			double curtime = evaltime();
			delta = curtime - lasttime;
			double time = curtime - starttime;			

			if (!inited)
			{
				nos::reset_terminal();
				nos::println("Init plugin for", this->model->GetName());
				inited = 1;

				auto joint0 = model->GetJoint("joint0");
				auto joint1 = model->GetJoint("joint1");
				auto joint2 = model->GetJoint("joint2");
				joint0->SetProvideFeedback(true);
				joint1->SetProvideFeedback(true);
				joint2->SetProvideFeedback(true);

				lasttime = curtime;

				return;
			}

			rabbit::htrans2<double> joint0_rot(-model->GetJoint("joint0")->Position(0), {0, 0});
			rabbit::htrans2<double> joint1_rot(-model->GetJoint("joint1")->Position(0), {0, 0});

			auto wrench = model->GetJoint("joint2")->GetForceTorque(0);
			auto force1 = wrench.body1Force;
			auto torque1 = wrench.body1Torque;

			linalg::vec<double,2> local_force = {-force1.X(), -force1.Z()};

			auto link0 = model->GetLink("link_0");
			auto pos0 = link0->WorldCoGPose().Pos();
			rabbit::htrans2<double> trans(0, {0, 0.8});

			auto joint0_pose = rabbit::htrans2<double>(0, {pos0.X(), pos0.Z()});
			auto joint1_pose = joint0_pose * joint0_rot * trans;
			auto output_pose = joint1_pose * joint1_rot * trans;

			linalg::vec<double,2> global_force = linalg::rot(output_pose.orient, local_force);

			auto sens = rabbit::screw<double, 2>(-1, {0, 0});

			auto joint0_sens =
			    joint0_pose.rotate(
			        sens.kinematic_carry((joint0_pose.inverse() * output_pose).center));
			auto joint1_sens =
			    joint1_pose.rotate(
			        sens.kinematic_carry((joint1_pose.inverse() * output_pose).center));

			int left = model->GetName() == "manip1";

			linalg::vec<double, 2> position_target;
			
			double X = 0.45 + 0.05 * time;
			if (X > 1.3) X = 1.3;
			position_target = {left ? -0.35 : 0.35, X};

			
			if (time < 5) 
			{
				position_target = {left ? -1. : 1., 1.2};
			}

			auto position_error = position_target - output_pose.translation();
			position_error += linalg::vec<double,2>{global_force.x, 0} * ForceKoeff;
			
			position_integral += position_error * delta;

			linalg::vec<double, 2> target;	
			if (time < 10) 
			{
				target = 
				1 * position_error +
				0.05 * position_integral 
				+ global_force * ForceKoeff;
			}
			else {
				auto to_cargo = CARGO_POSITION.center - output_pose.center;

				target = 
					to_cargo * 0.1 +
					CARGO_TARGET_VELOCITY.kinematic_carry(to_cargo).lin
					+ global_force * ForceKoeff;

				//nos::println(CARGO_TARGET_VELOCITY.kinematic_carry(-to_cargo).lin);
			}



			linalg::vec<double, 2> vectors[2] = { joint0_sens.lin, joint1_sens.lin };

			double coords[2];
			ralgo::svd_backpack<double, linalg::vec<double, 2>>(coords, target, vectors, 2);

			coords[0] = igris::clamp(coords[0], -3, 3);
			coords[1] = igris::clamp(coords[1], -3, 3);

			linalg::vec<double, 2> result = joint0_sens.lin * coords[0] + joint1_sens.lin * coords[1];
			auto control_error = linalg::length(target - result);


			//if (control_error < 1e-3)
			//{
				joint0_regulator.speed2_target = coords[0];
				joint1_regulator.speed2_target = coords[1];
			//}
			//else
			//{
			//	joint0_regulator.speed2_target = 0;
			//	joint1_regulator.speed2_target = 0;
			//}

			if (model->GetName() == "manip1")
			{
				//nos::println(model->GetName(), delta);
				//nos::println(force1.X(), force1.Y(), force1.Z(), global_force);
			}

			Control(model->GetJoint("joint0"), &joint0_regulator);
			Control(model->GetJoint("joint1"), &joint1_regulator);

			lasttime = curtime;
		}

		void Control(gazebo::physics::JointPtr joint, Regulator * reg)
		{
			double current_position = joint->Position(0);
			double current_speed = joint->GetVelocity(0);

			if (reg -> speed2_loop_enabled)
			{
				reg->position_target += reg->speed2_target * delta;
			}

			if (reg -> position_loop_enabled)
			{
				reg->position_error = reg->position_target - current_position;
				reg->position_integral += reg->position_error * delta;
				reg->speed_target =
				    reg->pos_kp * reg->position_error +
				    reg->pos_ki * reg->position_integral 
				    - reg->control_signal * ForceKoeff;
			}

			reg->speed_error = reg->speed_target - current_speed;
			reg->speed_integral += reg->speed_error * delta;
			reg->control_signal =
			    reg->spd_kp * reg->speed_error +
			    reg->spd_ki * reg->speed_integral;

			joint->SetForce(0, reg->control_signal);
		}

	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}