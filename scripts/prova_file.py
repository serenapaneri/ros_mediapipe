#! /usr/bin/env python3

import rospy
from docx import Document

found = 0

def main():

    global found
    rospy.init_node('prova_file')
    found += 1
    motion = True

    doc = Document()
    doc.add_heading('000' + str(found), 0)

    doc.add_heading('Consciousness/Unconsciousness', 1)

    table1 = doc.add_table(1, 2)
    cells0 = table1.rows[0].cells
    cells0[0].text = 'Motion'
    cells0[1].text = str(motion)

    cells1 = table1.add_row().cells
    cells1[0].text = 'hdoqbv'
    cells1[1].text = str(motion)
    cells2 = table1.add_row().cells
    cells2[0].text = 'kwwwwwwrj'
    cells2[1].text = str(motion)
    cells3 = table1.add_row().cells
    cells3[0].text = 'lsjgwbrjb'
    cells3[1].text = str(motion)


    doc.add_heading('Consciousness/Unconsciousness', 1)

    table2 = doc.add_table(1, 2)
    cells02 = table2.rows[0].cells
    cells02[0].text = 'Motion'
    cells02[1].text = str(motion)
    cells12 = table2.add_row().cells
    cells12[0].text = 'hdoqbv'
    cells12[1].text = str(motion)
    cells22 = table2.add_row().cells
    cells22[0].text = 'kwwwwwwrj'
    cells22[1].text = str(motion)
    cells32 = table2.add_row().cells
    cells32[0].text = 'lsjgwbrjb'
    cells32[1].text = str(motion)
    
    doc.add_page_break()

    doc.save('/home/spot/ros_ws/src/spot_mediapipe/results/provaaaaaaaaa{}.docx'.format(found))

    print(found)

    rospy.spin()

if __name__ == '__main__':
    main()
