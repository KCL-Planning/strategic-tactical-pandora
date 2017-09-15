/**
 * This file contains the parsing of the ActionFeedback topic and the logic in deciding
 * when to replan.  TODO: this parsed data will be stored directly in the ontology, and
 * not in the list of variables in PlanningEnvironment.h  At that time the logic will be
 * moved to an external module.
 * ROS calls feedbackCallback() in its own thread.
 * Mesages are farmed to separate methods based on action type.
 */


namespace PandoraKCL {

	/*---------------------------*/
	/* Specific action responses */
	/*---------------------------*/

	void feedbackGoto(const planning_msgs::ActionFeedback::ConstPtr& msg) {

		// actions failed
		if(0 == PandoraKCL::actionList[msg->action_id].name.compare("goto") && 0 == msg->status.compare("action failed")) {
			ROS_INFO("KCL: Movement failed; replanning.");
			PandoraKCL::replanRequested = true;
			PandoraKCL::actionCompleted[msg->action_id] = true;
		}

	}

	void feedbackObserve(const planning_msgs::ActionFeedback::ConstPtr& msg) {

		for(size_t i=0;i<PandoraKCL::actionList[msg->action_id].parameters.size();i++) {
			// remember which inspection points have been discovered
			diagnostic_msgs::KeyValue pair = PandoraKCL::actionList[msg->action_id].parameters[i];
			if(pair.key.find("ip_id")!=std::string::npos) {
				PandoraKCL::inspectionPointsInspected[pair.value] = true;
				PandoraKCL::inspectionPointsDiscovered[pair.value] = ( 0 == msg->status.compare("action achieved") );
			}
		}

		for(size_t i=0;i<msg->information.size();i++) {

			// check if AUV has discovered a panel
			if(msg->information[i].key.find("_position")!=std::string::npos) {

				// "panel_n_position" : "[double x,double y,double z, double q_x, double q_y,  double q_z, double q_w]" in world coordinates
				std::string panelID = msg->information[i].key.substr(6,msg->information[i].key.find("_position")-6);
				Point3DQuat pose;
				parsePoint3DwithQuaternion(pose, msg->information[i].value);

				PandoraKCL::panelPositions[panelID] = pose;
				PandoraKCL::panelExamined[panelID] = false;

				std::stringstream ss;
				ss << panelID;
				name_map[ss.str()] = ss.str();

				ROS_INFO("KCL: Detected panel %s; requesting replan", panelID.c_str());
				replanRequested = true;
			}

			// keep looking at almost detected panel
			if(msg->information[i].key.find("_in_fov")!=std::string::npos) {
				// "panel_n_in_fov" -> "true/false"
				if(0 == msg->information[i].value.compare("true")) {
					PandoraKCL::currentAction--;
					ROS_INFO("KCL: Possibly detected panel, so keep looking...");
				}
			}
		}

		// action completed
		if(!PandoraKCL::actionCompleted[msg->action_id] && 0 == msg->status.compare("timeout"))
			PandoraKCL::actionCompleted[msg->action_id] = true;
	}

	void feedbackValveState(const planning_msgs::ActionFeedback::ConstPtr& msg) {

		for(size_t i=0;i<msg->information.size();i++) {

			// "valve_n_angle" : "[double]"
			if(msg->information[i].key.find("_angle")!=std::string::npos) {
				std::string valveID = msg->information[i].key.substr(6,msg->information[i].key.find("_angle")-6);
				double angle = atof(msg->information[i].value.c_str());
				PandoraKCL::valveAngles[valveID] = angle;
				ROS_INFO("KCL: Valve %s detected at %f", valveID.c_str(), valveAngles[valveID]);

				// check valve deadline is correctly completed
				double dispatchTime = PandoraKCL::actionList[msg->action_id].dispatch_time + (planStart - missionStart);
				double valveGoal = PandoraKCL::getCurrentValveGoal(valveID, dispatchTime);
				double valveDeadline = PandoraKCL::getCurrentValveDeadline(valveID, dispatchTime);
				if(valveDeadline < 0) {
					// all is fine
					ROS_INFO("KCL: (VALVE_CHECK) Valve %s has no deadline, thank goodness", valveID.c_str());
				} else if(fabs(angle-valveGoal) < 0.2) {
					// all is fine
					lastCompletedDeadline[valveID] = valveDeadline;
					ROS_INFO("KCL: (VALVE_CHECK) Valve %s correct for next deadline!", valveID.c_str());
				} else if(lastCompletedDeadline[valveID] < valveDeadline) {
					// all is fine
					ROS_INFO("KCL: (VALVE_CHECK) Valve %s not correct for next deadline, but we plan to correct it!", valveID.c_str());
				} else {
					// all is not fine
					ROS_INFO("KCL: (VALVE_CHECK) Valve %s not correct and we have not planned to fix it. Replanning.", valveID.c_str());
					PandoraKCL::replanRequested = true;
				}
			}

			// "valve_n_in_panel" : "[int]"
			if(msg->information[i].key.find("_in_panel")!=std::string::npos) {
				std::string valveID = msg->information[i].key.substr(6,msg->information[i].key.find("_in_panel")-6);
				std::string panelID = msg->information[i].value;
				PandoraKCL::valves[valveID] = panelID;
				PandoraKCL::panelExamined[panelID] = true;
				std::stringstream ss;
				
				ROS_INFO("KCL: Valve %s detected at panel %s", valveID.c_str(), panelID.c_str());
				
				//ss << "v" << valveID;
				ss << valveID;
				name_map[ss.str()] = ss.str();
			}
		}

		for(size_t i=0;i<msg->information.size();i++) {

			// "panel_n_state" : "panel_missing"
			if(0 == msg->information[i].value.compare("panel_missing")) {

				// remove all knowledge of panels
				panelPositions.clear();
				panelReachability.clear();
				panelExamined.clear();
				valves.clear();
				valveAngles.clear();
				panelVisibility.clear();

				// we have drifted
				drifted = true;

				ROS_INFO("KCL: Panel detected as missing. Forget everything.");
				PandoraKCL::replanRequested = true;
				PandoraKCL::actionCompleted[msg->action_id] = true;
			}
		}

	}

	void feedbackTurnValve(const planning_msgs::ActionFeedback::ConstPtr& msg) {

		// track arm calibration
		// arm_calibration[PandoraKCL::actionList[msg->action_id].auv]++;

		// set valve state to turned; panel state to unexamined				
		std::string valveID = PandoraKCL::actionList[msg->action_id].parameters[0].value;
		PandoraKCL::panelExamined[valves[valveID]] = false;
		if(0 == msg->status.compare("action achieved"))
			lastCompletedDeadline[valveID] = (double)atof(PandoraKCL::actionList[msg->action_id].parameters[2].value.c_str());

		for(size_t i=0;i<msg->information.size();i++) {


			if(msg->information[i].key.find("_state")!=std::string::npos) {

				// "valve_n_state" : "valve_blocked"
				if(0 == msg->information[i].value.compare("valve_blocked")) {

					// increment blocked count
					if(PandoraKCL::valveBlocked.find(valveID) != PandoraKCL::valveBlocked.end())
						PandoraKCL::valveBlocked[valveID]++;
					else PandoraKCL::valveBlocked[valveID] = 1;
					ROS_INFO("KCL: Valve %s detected as blocked %i time(s)", valveID.c_str(), valveBlocked[valveID]);
					if(valveBlocked[valveID]>1) ROS_INFO("KCL: Ignoring Valve %s in future.", valveID.c_str());
				}
			}
		}

		// action failed
		if(0 == msg->status.compare("action failed") && !replanRequested && valveBlocked[valveID]<2) {
			PandoraKCL::replanRequested = true;
			PandoraKCL::actionCompleted[msg->action_id] = true;
		}

		if(0 == msg->status.compare("action failed")) {
			PandoraKCL::actionCompleted[msg->action_id] = true;
		}
	}

	/*------------------*/
	/* general feedback */
	/*------------------*/

	/**
	 * listen to and process actionFeedback topic
	 */
	void feedbackCallback(const planning_msgs::ActionFeedback::ConstPtr& msg) {

		ROS_INFO("KCL: Feedback received [%i,%s]", msg->action_id, msg->status.c_str());
		//if(PandoraKCL::currentAction != (unsigned int)msg->action_id)
		//	ROS_INFO("KCL: Unexpected action ID: %d; current action: %zu", msg->action_id, PandoraKCL::currentAction);

		// action enabled
		if(!PandoraKCL::actionReceived[msg->action_id] && (0 == msg->status.compare("action enabled")))
			PandoraKCL::actionReceived[msg->action_id] = true;
		
		// action completed (successfuly)
		if(!PandoraKCL::actionCompleted[msg->action_id] && 0 == msg->status.compare("action achieved"))
			PandoraKCL::actionCompleted[msg->action_id] = true;

		if(0 > msg->action_id) return;

		// action finished
		if(PandoraKCL::auv_busy[PandoraKCL::actionList[msg->action_id].auv] && (0 == msg->status.compare("action achieved") || 0 == msg->status.compare("action failed") || 0 == msg->status.compare("action timeout")))
			PandoraKCL::auv_busy[actionList[msg->action_id].auv] = false;

		// more specific feedback
		if(0 == PandoraKCL::actionList[msg->action_id].name.compare("goto")) feedbackGoto(msg);
		if(0 == PandoraKCL::actionList[msg->action_id].name.compare("observe")) feedbackObserve(msg);
		if(0 == PandoraKCL::actionList[msg->action_id].name.compare("examine_panel")) feedbackValveState(msg);
		if(0 == PandoraKCL::actionList[msg->action_id].name.compare("turn_valve")) feedbackTurnValve(msg);
		if(0 == PandoraKCL::actionList[msg->action_id].name.compare("recalibrate_arm"))
			arm_calibration[PandoraKCL::actionList[msg->action_id].auv]=0;
	}

} // close namespace
