# Commands

## CLI tools used throughout tutorial

- rospack = ros+pack(age) : provides information related to ROS packages

- roscd = ros+cd : changes directory to a ROS package or stack

- rosls = ros+ls : lists files in a ROS package

- roscp = ros+cp : copies files from/to a ROS package

  - Example:
  
        roscp rospy_tutorials AddTwoInts.srv srv/

- rosmsg = ros+msg : provides information related to ROS message definitions

  - used with show

- rossrv = ros+srv : provides information related to ROS service definitions

  - used with show

- catkin_make : makes (compiles) a ROS package

- rosmake = ros+make : makes (compiles) a ROS package (if you're not using a catkin workspace)

## C++ classes and methods used throughout tutorial

- Handle for ROS Node

      ros::NodeHandle n;

- Get a service with name 'add_two_ints' which uses signature of srv/AddTwoInts.srv

      ros::ServiceClient client = n.serviceClient<my_pack::AddTwoInts>("add_two_ints");

- Try to fill the response for request. Returns true on success, else false,

      client.call(srv);

- Tell the master node that service add_two_ints is available

       ros::ServiceServer service = n.advertiseService("add_two_ints", add);

  It uses the function add with signature:

      bool add(my_pack::AddTwoInts::Request  &req,
         my_pack::AddTwoInts::Response &res)

- Run a loop which replies to the requests one-by-one

       ros::spin();

- An option of getting callbacks if you're already in a loop.

      ros::spinOnce();

- Subscriber class which subsribes to "chatter" topic, sets the queue size to 1000 and sets the callback to *chatterCallback*

      ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  Callback has the following signature:

       void chatterCallback(const std_msgs::String::ConstPtr& msg);

- Publisher class which tells the ROS Master that "chatter" topic is published to and sets the queue size to 1000.

       ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

- Rate of the loop 10Hz
  
      ros::Rate loop_rate(10);

  Used with:

      loop_rate.sleep();

  in a loop which sleeps the rest of the left time (if there is any)

- Publish the message

      chatter_pub.publish(msg);

## C++ functions used throughout tutorial

- Init the node with name 'name' which supports remapping
  
      ros::init(argc, argv, "add_two_ints_client");

- Logger for stdout and rosout

      ROS_INFO("I'm the logger");

- Checks if node is in consistent state

      ros::ok();

## Python classes used throughout tutorial

## Python functions used throughout tutorial
  
