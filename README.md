# rviz_template

이 레포지토리는 ROS1 melodic (18.04)와 Python 2.7을 이용하여 Rviz를 사용하기 위한 모듈입니다. 점과 선, 원(기둥), 텍스트 등의 마커를 삽입/삭제하는 기능을 담고 있습니다.

## Usage
1. roslaunch : 마커를 생성하는 노드(visual_rviz_node)와 이를 subscribe해 시각화를 하는 rviz 노드(rviz_node)를 실행하고, 노드 관계를 표현하는 rqt graph를 작동시킵니다.
    ```
    roslaunch rviz_template example.launch
    ```
2. rosrun: 각 노드를 따로 작동시킵니다. 아래의 명령어는 각 터미널에서 따로 실행해야 합니다.
    ```
    roscore
    rosrun rviz_template usage_example.py
    rviz -d [rviz 파일 경로]/rviz_config.rviz
    ```

![](rqt_graph.png)

![](rviz_example.png)
