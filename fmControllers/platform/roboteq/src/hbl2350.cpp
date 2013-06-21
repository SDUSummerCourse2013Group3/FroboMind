#include "roboteq/hbl2350.hpp"

hbl2350::hbl2350( )
:local_node_handler("~"),global_node_handler()
{
	// Two channel operation
	two_channel = true;

	// Setup Channels
	ch1.ch = LEFT;
	ch2.ch = RIGHT;

	// Register callbacks
	ch1.transmit_cb = new CallbackHandler<hbl2350>(this,&hbl2350::transmit);
	ch2.transmit_cb = new CallbackHandler<hbl2350>(this,&hbl2350::transmit);
	ch1.init_cb = new CallbackHandler<hbl2350>(this,&hbl2350::initController);
	ch2.init_cb = new CallbackHandler<hbl2350>(this,&hbl2350::initController);

	//Motor controller constant open loop max outputmax_output
	ch1.roboteq_max = 1000;
	ch2.roboteq_max = 1000;

	// Initialise status
	status.cmd_vel_publishing = status.deadman_pressed = status.initialised = status.online = status.responding = false;
	status.emergency_stop = true;

	// Declare variables for parsing parameters
	double max_time_diff_input;
	std::string cmd_vel_ch1_topic, cmd_vel_ch2_topic, serial_tx_topic, serial_rx_topic, command_relay_topic, deadman_topic,
	encoder_ch1_topic, encoder_ch2_topic, power_ch1_topic, power_ch2_topic, status_topic, temperature_topic;

	// Parse from parameter server
	local_node_handler.param<std::string>("cmd_vel_ch1_topic", cmd_vel_ch1_topic, "/fmActuators/cmd_vel_ch1");
	local_node_handler.param<std::string>("cmd_vel_ch2_topic", cmd_vel_ch2_topic, "/fmActuators/cmd_vel_ch2");
	local_node_handler.param<std::string>("serial_rx_topic", serial_rx_topic, "/fmCSP/S0_rx");
	local_node_handler.param<std::string>("serial_tx_topic", serial_tx_topic, "/fmCSP/S0_tx");
	local_node_handler.param<std::string>("command_relay_topic", command_relay_topic, "/fmData/command");
	local_node_handler.param<std::string>("deadman_topic", deadman_topic, "/fmHMI/joy");
	local_node_handler.param<std::string>("encoder_ch1_topic", encoder_ch1_topic, "/fmSensors/encoder_ch1");
	local_node_handler.param<std::string>("encoder_ch2_topic", encoder_ch2_topic, "/fmSensors/encoder_ch2");
	local_node_handler.param<std::string>("power_ch1_topic", power_ch1_topic, "/fmSensors/power_ch1");
	local_node_handler.param<std::string>("power_ch2_topic", power_ch2_topic, "/fmSensors/power_ch2");
	local_node_handler.param<std::string>("status_topic", status_topic, "/fmActuators/status");
	local_node_handler.param<std::string>("temperature_topic", temperature_topic, "/fmActuators/temperature");

	// Init channel parameters
	local_node_handler.param<double>("p_gain", ch1.p_gain, 1);
	ch2.p_gain = ch1.p_gain;
	local_node_handler.param<double>("i_gain", ch1.i_gain, 0);
	ch2.i_gain = ch1.i_gain;
	local_node_handler.param<double>("d_gain", ch1.d_gain, 0);
	ch2.d_gain = ch1.d_gain;
	local_node_handler.param<double>("i_max",ch1.i_max,50);
	ch2.i_max = ch1.i_max;

	local_node_handler.param<double>("/robot_max_velocity",ch1.max_velocity_mps,1.0);
	ch2.max_velocity_mps = ch1.max_velocity_mps;
	local_node_handler.param<double>("max_controller_command",ch1.max_output,300);
	ch2.max_output = ch1.max_output;
	if(ch1.max_output > ch1.roboteq_max) ch1.max_output = ch1.roboteq_max;
	if(ch2.max_output > ch2.roboteq_max) ch2.max_output = ch2.roboteq_max;

	double tmp;
	local_node_handler.param<double>("ticks_per_meter",tmp,1285.0);
	ch1.ticks_to_meter = -(1.0/tmp);
//	ch2.ticks_to_meter = (1.0/tmp);
	ch2.ticks_to_meter = (tmp/682.0)/tmp; // TODO: Crappy 1.88 RoboTeQ hack

	ch1.last_deadman_received = ch2.last_deadman_received = ros::Time::now();
	ch1.velocity = ch2.velocity = 0;
	ch1.regulator.set_params(ch1.p_gain , ch1.i_gain , ch1.d_gain ,ch1.i_max , ch1.roboteq_max);
	ch2.regulator.set_params(ch2.p_gain , ch2.i_gain , ch2.d_gain ,ch2.i_max , ch2.roboteq_max);

	// Init general parameters
	local_node_handler.param<double>("max_time_diff",max_time_diff_input,0.5);
	max_time_diff = ros::Duration(max_time_diff_input);
	last_serial_msg = ros::Time::now();
	local_node_handler.param<bool>("closed_loop_operation", closed_loop_operation, false);

	// Setup publishers
	setSerialPub( local_node_handler.advertise<msgs::serial>( serial_tx_topic,10 ));
	setEncoderCh1Pub( local_node_handler.advertise<msgs::IntStamped>( encoder_ch1_topic, 10));
	setEncoderCh2Pub( local_node_handler.advertise<msgs::IntStamped>( encoder_ch2_topic, 10));
	setPowerCh1Pub( local_node_handler.advertise<msgs::IntStamped>( power_ch1_topic, 10));
	setPowerCh2Pub( local_node_handler.advertise<msgs::IntStamped>( power_ch2_topic, 10));
	setStatusPub( local_node_handler.advertise<msgs::StringStamped>( status_topic, 10));
	ch1.setStatusPub( local_node_handler.advertise<msgs::StringStamped>( status_topic, 10));
	ch2.setStatusPub( local_node_handler.advertise<msgs::StringStamped>( status_topic, 10));
	setTemperaturePub( local_node_handler.advertise<msgs::StringStamped>( temperature_topic, 10));

	// Set up subscribers
	serial_sub = local_node_handler.subscribe<msgs::serial>(serial_rx_topic,10,&hbl2350::onSerial,this);
	ch1.cmd_vel_sub = local_node_handler.subscribe<geometry_msgs::TwistStamped>(cmd_vel_ch1_topic,10,&Channel::onCmdVel,&ch1);
	ch2.cmd_vel_sub = local_node_handler.subscribe<geometry_msgs::TwistStamped>(cmd_vel_ch2_topic,10,&Channel::onCmdVel,&ch2);
	deadman_sub = local_node_handler.subscribe<std_msgs::Bool>(deadman_topic,10,&hbl2350::onDeadman,this);
}

void hbl2350::spin(void)
{
	// Wait for RoboTeQ to come online
	ros::Rate r(5);
	while(!this->subscribers())
	{
		ROS_INFO_THROTTLE(1,"Waiting for serial node to subscribe");
		r.sleep();
	}
	r.sleep();

	// Initialize timer
	ros::Timer t = global_node_handler.createTimer(ros::Duration(0.02),&hbl2350::onTimer,this);

	ros::spin();
}

void hbl2350::updateStatus(void)
{
	// Update time based status variables.
	status.deadman_pressed = ((ros::Time::now() - ch1.last_deadman_received) < max_time_diff) || ((ros::Time::now() - ch2.last_deadman_received) < max_time_diff);
	status.cmd_vel_publishing = ( (ros::Time::now() - ch1.last_twist_received) < max_time_diff) || ( (ros::Time::now() - ch2.last_twist_received) < max_time_diff);
	status.responding = ((ros::Time::now() - last_serial_msg) < max_time_diff);
}

void hbl2350::initController(std::string config)
{
	/*
	 * Temporary simple implementation. Should in time be implemented as a config file parser. In time...
	 */
	ROS_INFO("Initializing...");

	sleep(1);
	transmit(2,	"^ECHOF", 1 ); sleep(TIME_BETWEEN_COMMANDS);						// Echo is disabled
	transmit(1,	"# C"); sleep(TIME_BETWEEN_COMMANDS);								// Clear buffer
	transmit(1,	"?P"); sleep(TIME_BETWEEN_COMMANDS);								// Request power readings
	transmit(1,	"?V"); sleep(TIME_BETWEEN_COMMANDS);								// Request voltage readings
	transmit(1,	"?T"); sleep(TIME_BETWEEN_COMMANDS);								// Request temperature readings
	transmit(1,	"?FS"); sleep(TIME_BETWEEN_COMMANDS);								// Request status flag
	transmit(1, "?FF"); sleep(TIME_BETWEEN_COMMANDS);								// Request fault flag
	transmit(1, "?CB"); sleep(TIME_BETWEEN_COMMANDS);								// Request absolute hall count
	transmit(1,	"# 50" ); sleep(TIME_BETWEEN_COMMANDS);							    // Repeat buffer every 10 ms
	transmit(1,	"^ALIM 1 70" ); sleep(TIME_BETWEEN_COMMANDS);
	transmit(1,	"^ALIM 2 70" ); sleep(TIME_BETWEEN_COMMANDS);
	sleep(2);

	ROS_INFO("Initialization finished");
	status.initialised = true;
	//status.responding = false;
}

