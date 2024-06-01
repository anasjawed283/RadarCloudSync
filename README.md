<h1><b>RadarCloudSync</b></h1> 

<h4>RadarCloudSync is a radar system project that transmits data to the cloud and supports real-time data fetching for instant analysis and monitoring. This repository provides instructions on setting up the environment, adding the necessary code, and linking it with ThingSpeak for cloud integration.</h4>

<!-- ![image](https://media.giphy.com/media/gUCDHxamnl2CLYD5Dv/giphy.gif?cid=790b7611v83fip4nsnx2tv0i2g5a7enc6htmamxynxu6hc4c&ep=v1_stickers_search&rid=giphy.gif&ct=s) -->
## Prerequisites

- Ubuntu 20.04 or later
- ROS Noetic
- Git
- Python 3
- ThingSpeak account


# Setting up ThingSpeak

- Create a thingspeak account
  
  ![image](https://github.com/anasjawed283/RadarCloudSync/assets/103234658/f7530639-0aa2-40a5-8531-2c73541b7eca)

- Click on new channel
  
  ![image](https://github.com/anasjawed283/RadarCloudSync/assets/103234658/5ce4e8f3-8201-4100-aa67-f736ae3923cd)
  
- Add field and other information

  ![image](https://github.com/anasjawed283/RadarCloudSync/assets/103234658/d52bccab-f0f0-4c2d-b07b-f9f81e4c8599)

- Click on save channel
  
  ![image](https://github.com/anasjawed283/RadarCloudSync/assets/103234658/21ee04af-1b80-4b21-92c4-1460945308d8)

- You get a screen as this
  
  ![image](https://github.com/anasjawed283/RadarCloudSync/assets/103234658/46aba297-2075-4848-bf5b-97243acb8f25)

- Click on API Keys

  ![image](https://github.com/anasjawed283/RadarCloudSync/assets/103234658/0ffbcb9c-4f55-4945-b326-eec01a8e62f3)

- Replace your read and write api key and the channel id at the sketch.ino code and ros code.




# Setting up the simulation

- Create an account on workwi if not there visit 👉 [Workwi Dashboard ](https://wokwi.com/dashboard/projects)
 
- Click on new Project
  
  ![image](https://github.com/anasjawed283/RadarCloudSync/assets/103234658/a1468955-5835-4adb-90a1-aea3a2cb87de)
 
- Click on blank project
  
  ![image](https://github.com/anasjawed283/RadarCloudSync/assets/103234658/d7f4080d-608e-486d-9451-01c4209746a8)

- Copy paste the sketch.ino code the diagram.json code and add the libraries as in this repository

  ![image](https://github.com/anasjawed283/RadarCloudSync/assets/103234658/95767b09-ca32-4dcc-9407-2f4245797ac0)

- To start the simulation click on run button

  ![image](https://github.com/anasjawed283/RadarCloudSync/assets/103234658/82fbf08e-4bfb-4470-8b2a-a60c95900ea5)

- Wait until WiFi is connected if not connecting within 30 seconds recheck the previous steps

  ![image](https://github.com/anasjawed283/RadarCloudSync/assets/103234658/59f38941-1ba2-4db5-93d2-488975c725ae)

- Click on the Ultrasonic sensor and drag the slider to change the distance value

  ![image](https://github.com/anasjawed283/RadarCloudSync/assets/103234658/48716391-5f0d-4630-afe4-4cb981ef2b82)

- Check the Thingspeak channel the data is recieved

  ![image](https://github.com/anasjawed283/RadarCloudSync/assets/103234658/20d7d6d9-b13a-409d-b4d4-d23e9dd586b1)



# ROS Environment setup

### Creating a Catkin Workspace

1. Open a terminal and create a new directory for your catkin workspace:
    ```bash
    mkdir RadarCloudSync
    cd RadarCloudSync
    mkdir src
    catkin_make
    ```

2. Source the workspace:
    ```bash
    source devel/setup.bash
    ```

3. Creating new package:
    ```bash
    cd src
    catkin_create_pkg new_pkg rospy roscpp
    ```

4. Adding the code:
    ```bash
    gedit ROS_Publisher.py
    ```
<b><i>paste the code make changes and Ctrl S</i></b>

5. Running the code:
    ```bash
    chmod +x ROS_Publisher.py
    cd ../..
    source devel/setup.bash
    rosrun new_pkg ROS_Publisher.py
    ```


## Contributing

Contributions are welcome! Please fork this repository and submit a pull request with your changes. For major changes, please open an issue first to discuss what you would like to change.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---
