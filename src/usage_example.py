#!/usr/bin/env python
# -*- coding:utf-8 -*-

#######################################################################
# Copyright (C) 2022 EunGi Han(winterbloooom) (winterbloooom@gmail.com)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>


import rospy
import rviz_visualizer as visual
from visualization_msgs.msg import MarkerArray

# rviz publisher
rviz_pub = rospy.Publisher("/visual_rviz", MarkerArray, queue_size=0)

# make marker ids list for convenience
# 마커에 일일히 마커 번호를 상수를 부여하면 중복 등 혼선이 있을 수 있음
# 따라서 id 리스트를 만들어두고 이를 부여함
ids = list(range(0, 100))

# One Point (포인트 한 개)
point = visual.point_rviz(
    name="point", id=ids.pop(), point=[1, 1], color_r=245, color_g=102, color_b=66, scale=0.2
)

# Points (set of Points) (포인트 여러 개)
points_positions = [[1, 0], [2, 0], [3, 0], [4, 1], [5, 2], [6, 3]]
points = visual.points_rviz(
    name="points",
    id=ids.pop(),
    points=points_positions,
    color_r=245,
    color_g=218,
    color_b=66,
    scale=0.2,
)

# arrow (화살표)
arrow = visual.arrow_rviz(
    name="arrow", id=ids.pop(), tail=[6, 3], head=[1, 0], color_r=156, color_g=245, color_b=66
)

# text (텍스트)
text = visual.text_rviz(name="text", id=ids.pop(), position=[1, 0], text="Hello\nWorld!")

# linelist (라인리스트)
line_points = [[-1, -1], [-2, -2], [-3, -3], [-4, -3], [-5, -3], [-6, -3]]
linelist = visual.linelist_rviz(
    name="linelist",
    id=ids.pop(),
    lines=line_points,
    color_r=203,
    color_g=33,
    color_b=237,
    scale=0.2,
)

# cylinder (원기둥, 원)
cylinder = visual.cylinder_rviz(
    name="cylinder", id=ids.pop(), center=[1, 1], scale=3, color_r=224, color_g=101, color_b=101
)

# make array with markers (만든 마커를 리스트로 만듦)
all_markers = visual.marker_array_rviz([point, points, arrow, text, linelist, cylinder])

# make another point
point2 = visual.point_rviz(
    name="point", id=ids.pop(), point=[2, 2], color_r=245, color_g=102, color_b=66
)

# append a mark to marker array (마커 하나를 마커 리스트에 추가함)
visual.marker_array_append_rviz(all_markers, point2)


def main():
    rospy.init_node("visual_rviz_node", anonymous=False)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rviz_pub.publish(all_markers)
        rate.sleep()


if __name__ == "__main__":
    main()
