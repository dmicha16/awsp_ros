    /*  Example code to use the additional logger function,
        that allows logging passed arguments */
    
    std::string test_file = "test_file.csv";
    std::string file_data_1; 
    std::string file_data_2; 
    std::string file_data_3; 
    std::string file_data_full; 

    for (int i = 0; i < 5; i ++)
    {
        file_data_1 = std::to_string (rand() % 100 + 1);
        file_data_2 = std::to_string (rand() % 100 + 1);
        file_data_3 = std::to_string (rand() % 100 + 1);
        file_data_full = file_data_1 + ", " + file_data_2 + ", " + file_data_3;

        logger.additional_logger(file_data_full, test_file);
    }

    /* Example code to send instructions to the logger*/

    awsp_msgs::LogInstruction log_instruction;
    ros::init(argc, argv, "test_node");
    ROS_INFO("Node initialized!");
    ros::NodeHandle n;
    // PropellerTest test(n, 17, 27); // Left and Right pins respectively 
    ros::Publisher start_log = n.advertise<awsp_msgs::LogInstruction>("log_instruction", 1000);
    

    while (ros::ok())
    {
        ROS_INFO("------ Choose instruction for logging ------");
        ROS_INFO("--------------------------------------------");
        ROS_INFO("------ 0 to stop logging -------------------");
        ROS_INFO("------ 1 to start logging ------------------");
        ROS_INFO("------ 2 to log only GNSS data -------------");
        ROS_INFO("------ 0 to log only IMU data --------------");

        std::cin>> log_instruction.instruction;
        start_log.publish(log_instruction);
    }

    return 0;