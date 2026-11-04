from multiprocessing.connection import wait
from time import sleep
from traceback import print_tb
from controllerGui import *
import sys
from PyQt5.QtWidgets import *
from PyQt5 import QtWidgets, QtGui
import sqlite3
from PyQt5.QtGui import QIcon
from sensor_msgs.msg import JointState
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.node import Node
from rclpy.duration import Duration
import math


#MainWindow.show()
app = QtWidgets.QApplication(sys.argv)
MainWindow = QtWidgets.QMainWindow()
ui = Ui_MainWindow()
ui.setupUi(MainWindow)
MainWindow.show()


connection = sqlite3.connect("first_database4.db")
operation = connection.cursor()

connection.commit()
table = operation.execute("create table if not exists joints (Command string, L1 float, L2 float, L3 float, L4 float, L5 float, L6 float)")
connection.commit()

class CoordinateListener(Node):
    def __init__(self):

        super().__init__('Coordinate')
        self.tf_buffer = Buffer(cache_time=Duration(seconds = 10.))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.01, self.get_transform)
        self.trans = None

    def get_transform(self):
        
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('base_link', 'tool0', now) #timeout=Duration(seconds=1.0)
            self.trans = trans

            w = trans.transform.rotation.w
            x = trans.transform.rotation.x
            y = trans.transform.rotation.y
            z = trans.transform.rotation.z

            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
        
            ui.lcdNumber.display(round(trans.transform.translation.x,3))
            ui.lcdNumber_2.display(round(trans.transform.translation.y,3))
            ui.lcdNumber_3.display(round(trans.transform.translation.z,3))
            ui.lcdNumber_4.display(round(roll_x*57.2957795131,1))
            ui.lcdNumber_5.display(round(pitch_y*57.2957795131,1))
            ui.lcdNumber_6.display(round(yaw_z*57.2957795131,1))
        except:
            pass
            #ui.statusbar.showMessage("Cannot reach TF2 data")
            

coordinate_listener = CoordinateListener()
ui.executor.add_node(coordinate_listener)

def thread_run(): 
    t1= Thread(target=run_program)
    t1.start()

def thread_selected():
    t3= Thread(target=joint_selected)
    t3.start()    

def add_step():
    try:
        joint_state_subscriber = JointStateSubscriber()
        print("classsa şooldu")
        rclpy.spin_once(joint_state_subscriber)
        print("spin")
        joint_state_subscriber.destroy_node()
        
    except:
        print("çaliştirilamadi")
        joint_state_subscriber.destroy_node()
        ui.statusbar.showMessage("Cannot Add")

def add_cmd():
    if ui.addCmdCombo.currentIndex() == 0:  
        operation.execute("insert into joints (Command,L1,L2,L3,L4,L5,L6) values('Wait',1,0,0,0,0,0)")
        connection.commit()
        print("Succesful")
        list_table()

def insert_db():
    rowCount = ui.tableWidget.model().rowCount()
    columnCount = ui.tableWidget.model().columnCount()
    connection.execute("DELETE FROM joints;") #connection.execute("VACUUM;")
    
    for row in range(rowCount):
        rowData = []
        for column in range(columnCount):
            widgetItem =ui.tableWidget.item(row,column)
            if(widgetItem and widgetItem.text()):
                rowData.append((widgetItem.text()))
            else:
                rowData.append('')
         #print(rowData)
        query = " insert into joints (Command, L1,L2,L3,L4,L5,L6) values(?,?,?,?,?,?,?);"

        connection.execute(query,rowData)
        connection.commit()
    list_table()
    

def move_up():
    chosen_row=ui.tableWidget.currentRow()
    chosen_row2=ui.tableWidget.currentRow()
    chosen_column=ui.tableWidget.currentColumn()
    columnCount = ui.tableWidget.model().columnCount()
    print(chosen_row)
    print(chosen_column)
    rowData = [0,0,0,0,0,0,0]

    for column in range(0,columnCount):
        widgetItem =ui.tableWidget.item(chosen_row,column)
        if(widgetItem and widgetItem.text()):
            rowData[column] = widgetItem.text() #rowData.append(widgetItem.text())   
    rowData_2 = [0,0,0,0,0,0,0]

    for column in range(0,columnCount):
        
        widgetItem =ui.tableWidget.item(chosen_row2-1,column)
        if(widgetItem and widgetItem.text()):
            rowData_2[column]= widgetItem.text() #rowData_2.append(widgetItem.text())

    rowData_tmp=[0,0,0,0,0,0,0]
    rowData_tmp = rowData
    rowData = rowData_2
    rowData_2 = rowData_tmp
    print(rowData)
    print(rowData_2)
    columnCount = ui.tableWidget.model().columnCount()

    for column in range(0,columnCount):
        ui.tableWidget.setItem(chosen_row,column,QTableWidgetItem(str(rowData[column])))
    #rowData_2 = rowData_tmp
    for column in range(0,columnCount):
        ui.tableWidget.setItem(chosen_row2-1,column,QTableWidgetItem(str(rowData_2[column])))
    insert_db()

def move_down():
    chosen_row=ui.tableWidget.currentRow()
    chosen_row2=ui.tableWidget.currentRow()
    chosen_column=ui.tableWidget.currentColumn()
    columnCount = ui.tableWidget.model().columnCount()
    print(chosen_row)
    print(chosen_column)
    rowData = [0,0,0,0,0,0,0]

    for column in range(0,columnCount):
        widgetItem =ui.tableWidget.item(chosen_row,column)
        if(widgetItem and widgetItem.text()):
            rowData[column] = widgetItem.text() #rowData.append(widgetItem.text())   
    rowData_2 = [0,0,0,0,0,0,0]

    for column in range(0,columnCount):
        
        widgetItem =ui.tableWidget.item(chosen_row2+1,column)
        if(widgetItem and widgetItem.text()):
            rowData_2[column]= widgetItem.text() #rowData_2.append(widgetItem.text())

    rowData_tmp=[0,0,0,0,0,0,0]
    rowData_tmp = rowData
    rowData = rowData_2
    rowData_2 = rowData_tmp
    print(rowData)
    print(rowData_2)
    columnCount = ui.tableWidget.model().columnCount()

    for column in range(0,columnCount):
        ui.tableWidget.setItem(chosen_row,column,QTableWidgetItem(str(rowData[column])))
    #rowData_2 = rowData_tmp
    for column in range(0,columnCount):
        ui.tableWidget.setItem(chosen_row2+1,column,QTableWidgetItem(str(rowData_2[column])))
    insert_db()


def delete_step():

    chosen_step=ui.tableWidget.currentRow()
    deleted_item = str(chosen_step+1)
    print(deleted_item)
    query = "delete from joints where rowid = ? ;"
    vacuum="VACUUM ;"
    try:
        connection.commit()
        operation.execute(query,(deleted_item,))
        connection.commit()
        operation.execute(vacuum)
        connection.commit()
        ui.statusbar.showMessage("Deleted")
        list_table()
    except:
        ui.statusbar.showMessage("Cannot Delete")

def list_table():
    try:
        ui.tableWidget.clear()
        ui.tableWidget.setHorizontalHeaderLabels(("Cmd","L1","L2","L3","L4","L5","L6"))
        #ui.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        query = "select * from joints"
        operation.execute(query)

        for IndexSatir, KayitNumarasi in enumerate(operation):
            for indexSutun, KayitSutun in enumerate(KayitNumarasi):
                ui.tableWidget.setItem(IndexSatir,indexSutun,QTableWidgetItem(str(KayitSutun)))
        
        print(ui.tableWidget.model().rowCount())
    except:
        ui.statusbar.showMessage("Cannot List")
   
def read():
    index = ui.tableWidget.currentRow()
    print(index)
    #row = index.row()
    #newIndex = ui.tableWidget.model().index(row,0)
    #name = ui.tableWidget.model().itemData(newIndex)
    print("Index is :", ui.tableWidget.item(index,1).text())

def joint_selected():
        node = Node("ex_joint_goal")
        index = ui.tableWidget.currentRow()
        for x in range(0,6):
            ui.tableWidget.item(index,x).setBackground(QtGui.QColor(255, 140, 0))
        node.declare_parameter(
            "joint_positions",
            [
            float(ui.tableWidget.item(index,1).text()),
            float(ui.tableWidget.item(index,2).text()),
            float(ui.tableWidget.item(index,3).text()),
            float(ui.tableWidget.item(index,4).text()),
            float(ui.tableWidget.item(index,5).text()),
            float(ui.tableWidget.item(index,6).text()),

            ],
        )

        callback_group = ReentrantCallbackGroup()

        moveit2 = MoveIt2(
            node=node,
            joint_names=sr80.joint_names(),
            base_link_name=sr80.base_link_name(),
            end_effector_name=sr80.end_effector_name(),
            group_name=sr80.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )

        ui.executor.add_node(node)

        joint_positions = (
            node.get_parameter("joint_positions").get_parameter_value().double_array_value
        )
        node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")

        moveit2.move_to_configuration(joint_positions)
        moveit2.wait_until_executed()  
        for x in range(0,6):
            ui.tableWidget.item(index,x).setBackground(QtGui.QColor(60, 60, 60))  
        ui.executor.remove_node(node)
        node.destroy_node()

def run_program():
    if ui.runButton.isChecked():
        while 1:
            for i in range(0,12):
                if ui.runButton.isChecked():
                    try: 
                        if ui.tableWidget.item(i,0).text() == "Wait": 
                            for x in range(0,7):
                                ui.tableWidget.item(i,x).setBackground(QtGui.QColor(255, 140, 0))
                            sleep(float(ui.tableWidget.item(i,1).text()))
                            for x in range(0,7):
                                ui.tableWidget.item(i,x).setBackground(QtGui.QColor(60, 60, 60))

                        elif ui.tableWidget.item(i,0).text() == "Move":                     
                            if ui.runButton.isChecked():        
                                try:
                                    node = Node("ex_joint_goal")
                                    for x in range(0,7):
                                        ui.tableWidget.item(i,x).setBackground(QtGui.QColor(255, 140, 0))
                                    node.declare_parameter(
                                        "joint_positions",
                                        [
                                        float(ui.tableWidget.item(i,1).text()),
                                        float(ui.tableWidget.item(i,2).text()),
                                        float(ui.tableWidget.item(i,3).text()),
                                        float(ui.tableWidget.item(i,4).text()),
                                        float(ui.tableWidget.item(i,5).text()),
                                        float(ui.tableWidget.item(i,6).text()),

                                        ],
                                    )
                                    print("parameters declared")
                                    callback_group = ReentrantCallbackGroup()

                                    moveit2 = MoveIt2(
                                        node=node,
                                        joint_names=sr80.joint_names(),
                                        base_link_name=sr80.base_link_name(),
                                        end_effector_name=sr80.end_effector_name(),
                                        group_name=sr80.MOVE_GROUP_ARM,
                                        callback_group=callback_group,
                                    )

                                    ui.executor.add_node(node)
                                    print("executor")
                                    joint_positions = (
                                        node.get_parameter("joint_positions").get_parameter_value().double_array_value
                                    )
                                    node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")

                                    moveit2.move_to_configuration(joint_positions)
                                    moveit2.wait_until_executed()    
                                    ui.executor.remove_node(node)
                                    node.destroy_node() 
                                    for x in range(0,7):
                                        ui.tableWidget.item(i,x).setBackground(QtGui.QColor(60, 60, 60))      
                                    ui.statusbar.showMessage("destroyed")

                                except:
                                    node.destroy_node()
                                    ui.statusbar.showMessage("Cant Go")
                    except:
                        pass
         
    else:
        ui.statusbar.showMessage("Pause")
        
def button_clicked():                                          
    if ui.runButton.isChecked():
        ui.runButton.setIcon(QIcon('/home/cenk/Downloads/Icons/pause-icon-512.png'))                       
    else:
        ui.runButton.setIcon(QIcon('/home/cenk/Downloads/Icons/play-icon-512.png')) 

def moveRowsDown(): #not working
    columnCount = ui.tableWidget.model().columnCount()
    chosenRow = ui.tableWidget.currentRow()
    rowCount = ui.tableWidget.model().rowCount()
    #print(ui.tableWidget.item(chosenRow,0).text())
    for countRow in range (28, 0, -1):
        if ui.tableWidget.item(countRow,0).text()!='':
            newRowCount = countRow +1
            break
            
 
    print(newRowCount)
            
    for index in range (newRowCount-1, chosenRow-1,-1):
        rowData = [0,0,0,0,0,0,0]
        for column in range(0,columnCount):
            widgetItem =ui.tableWidget.item(index,column)
            if(widgetItem and widgetItem.text()):
                rowData[column] = widgetItem.text()


        for column in range(0,columnCount):
            ui.tableWidget.setItem(index+1,column,QTableWidgetItem(str(rowData[column])))

def copyRow():
    
    chosenRow = ui.tableWidget.currentRow()
    columnCount = ui.tableWidget.columnCount()
    global copiedRowData
    copiedRowData = [0,0,0,0,0,0,0]

    for column in range (0, columnCount):
        widgetItem =ui.tableWidget.item(chosenRow,column)
        if(widgetItem and widgetItem.text()):
            copiedRowData[column] = widgetItem.text()
    insert_db()
    ui.statusbar.showMessage("Row copied")
            

def pasteRow():
    moveRowsDown()
    chosenRow = ui.tableWidget.currentRow()
    columnCount = ui.tableWidget.columnCount()
    for column in range(0,columnCount):
        ui.tableWidget.setItem(chosenRow,column,QTableWidgetItem(str(copiedRowData[column])))
    insert_db()
    ui.statusbar.showMessage("Row Pasted")
    


class JointStateSubscriber(Node):

    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  
    
    def listener_callback(self, msg):
        try:    
            print("spin")
            print(msg)  
            L1_ADD = round(msg.position[0],3)
            L2_ADD = round(msg.position[1],3)
            L3_ADD = round(msg.position[2],3)
            L4_ADD = round(msg.position[3],3)
            L5_ADD = round(msg.position[4],3)
            L6_ADD = round(msg.position[5],3)
            print(L1_ADD)
        except:
            ui.statusbar.showMessage("Can't get joint data!!")
        try:
            add = "insert into joints (Command,L1,L2,L3,L4,L5,L6) values(?,?,?,?,?,?,?)"  
            operation.execute(add,('Move',L1_ADD,L2_ADD,L3_ADD,L4_ADD,L5_ADD,L6_ADD))
            connection.commit()
            ui.statusbar.showMessage("Succesful")
            list_table()
        except:
            ui.statusbar.showMessage("ERROR")


ui.retranslateUi(MainWindow)
ui.tabWidget.setCurrentIndex(0)
ui.horizontalSlider.sliderMoved['int'].connect(ui.spinBox.setValue) # type: ignore
ui.spinBox.valueChanged['int'].connect(ui.horizontalSlider.setValue) # type: ignore
ui.horizontalSlider_2.sliderMoved['int'].connect(ui.spinBox_2.setValue) # type: ignore
ui.spinBox_2.valueChanged['int'].connect(ui.horizontalSlider_2.setValue) # type: ignore
ui.horizontalSlider_3.sliderMoved['int'].connect(ui.spinBox_3.setValue) # type: ignore
ui.spinBox_3.valueChanged['int'].connect(ui.horizontalSlider_3.setValue) # type: ignore
ui.horizontalSlider_4.sliderMoved['int'].connect(ui.spinBox_4.setValue) # type: ignore
ui.horizontalSlider_5.sliderMoved['int'].connect(ui.spinBox_5.setValue) # type: ignore
ui.horizontalSlider_6.sliderMoved['int'].connect(ui.spinBox_6.setValue) # type: ignore
ui.spinBox_6.valueChanged['int'].connect(ui.horizontalSlider_6.setValue) # type: ignore
ui.spinBox_5.valueChanged['int'].connect(ui.horizontalSlider_5.setValue) # type: ignore
ui.spinBox_4.valueChanged['int'].connect(ui.horizontalSlider_4.setValue) # type: ignore
#ui.radioButton.clicked.connect(ui.label_4.setFocus) # type: ignore
#ui.radioButton.clicked.connect(ui.label_10.setFocus) # type: ignore
#ui.radioButton.clicked.connect(ui.label_8.setFocus) # type: ignore
#ui.radioButton.clicked.connect(ui.label_9.setFocus) # type: ignore
#ui.radioButton.clicked.connect(ui.label_11.setFocus) # type: ignore
#ui.radioButton.clicked.connect(ui.label_12.setFocus) # type: ignore
#ui.radioButton_2.clicked.connect(ui.label_4.setFocus) # type: ignore
#ui.radioButton_2.clicked.connect(ui.label_10.setFocus) # type: ignore
#ui.radioButton_2.clicked.connect(ui.label_9.setFocus) # type: ignore
#ui.radioButton_2.clicked['bool'].connect(ui.label_11.setFocus) # type: ignore
#ui.radioButton_2.clicked.connect(ui.label_12.setFocus) # type: ignore
ui.horizontalSlider_7.sliderMoved['int'].connect(ui.spinBox_7.setValue) # type: ignore
ui.horizontalSlider_8.sliderMoved['int'].connect(ui.spinBox_8.setValue) # type: ignore
ui.horizontalSlider_9.sliderMoved['int'].connect(ui.spinBox_9.setValue) # type: ignore
ui.horizontalSlider_10.sliderMoved['int'].connect(ui.spinBox_10.setValue) # type: ignore
ui.horizontalSlider_11.sliderMoved['int'].connect(ui.spinBox_11.setValue) # type: ignore
ui.horizontalSlider_12.sliderMoved['int'].connect(ui.spinBox_12.setValue) # type: ignore
ui.radioButton_2.toggled['bool'].connect(lambda: ui.label_4.setText("L1")) # type: ignore
ui.radioButton_2.toggled['bool'].connect(lambda: ui.label_10.setText("L2")) # type: ignore
ui.radioButton_2.toggled['bool'].connect(lambda: ui.label_8.setText("L3")) # type: ignore
ui.radioButton_2.toggled['bool'].connect(lambda: ui.label_9.setText("L4")) # type: ignore
ui.radioButton_2.toggled['bool'].connect(lambda: ui.label_11.setText("L5")) # type: ignore
ui.radioButton_2.toggled['bool'].connect(lambda: ui.label_12.setText("L6"))# type: ignore
ui.radioButton.toggled['bool'].connect(lambda: ui.label_4.setText("X")) # type: ignore
ui.radioButton.toggled['bool'].connect(lambda: ui.label_10.setText("Y")) # type: ignore
ui.radioButton.toggled['bool'].connect(lambda: ui.label_8.setText("Z")) # type: ignore
ui.radioButton.toggled['bool'].connect(lambda: ui.label_9.setText("Rx")) # type: ignore
ui.radioButton.toggled['bool'].connect(lambda: ui.label_11.setText("Ry")) # type: ignore
ui.radioButton.toggled['bool'].connect(lambda: ui.label_12.setText("Rz"))# type: ignore
ui.moveUpButton.clicked.connect(copyRow) # move_up
ui.moveDownButton.clicked.connect(pasteRow) #move_down
#ui.moveDownButton.clicked.connect(insert_db)
ui.runButton.clicked.connect(thread_run) #run_program
ui.runButton.clicked.connect(button_clicked) #run_program
ui.deleteButton.clicked.connect(delete_step)
ui.openDbButton.clicked.connect(list_table)
ui.addCmdButton.clicked.connect(add_cmd)
ui.servoAddButton.clicked.connect(add_step)
ui.executeButton.clicked.connect(thread_selected)

sys.exit(app.exec_())