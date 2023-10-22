#!/usr/bin/env python3
import sys
import rospy
import qdarktheme
import pyautogui
from pathlib import Path
from PySide6.QtWidgets import QMainWindow, QApplication
from PySide6.QtGui import QPixmap, QIcon, QFont


from rake_gui.tika_intro import Ui_MainWindow_intro #ui->py #TÜRKÇE
from TIKA_Main import window2

p = Path(__file__).parents[2]
archive_path = p / "src" / "gui_archive"
width, height= pyautogui.size()


class window1(QMainWindow):
    def __init__(self,app):
        super(window1, self).__init__()
    
        self.intro = Ui_MainWindow_intro()
        self.intro.setupUi(self)
        self.button1 = self.intro.pushButton
        self.button2 = self.intro.pushButton_2
        self.app = app
        
        self.setWindowTitle("Giriş Sayfası")
         
        # VISUALIZATION
        # Pics directory: /home/(user_name)/rake_ws/src/rake_packages_22_23/rake_gui/src/gui_archive/ 
        # Loading logos and pictures on Qt Designer created issues related to the directories
        # re-adding them on code is necessary.

        self.intro.label_2.setPixmap(QPixmap(str(archive_path/"rake_logo.png")))

        self.icon = QIcon()
        self.icon.addFile(str(archive_path/"rake_logo.png"))
        self.button1.setIcon(self.icon)
        self.button2.setIcon(self.icon)

        self.icon1 = QIcon()
        self.icon1.addFile(str(archive_path/"info_icon.jpg"))
        self.intro.tabWidget.addTab(self.intro.tab_1, self.icon1, "RAKE")

        self.icon2 = QIcon()
        self.icon2.addFile(str(archive_path/"social_icon.png"))
        self.intro.tabWidget.addTab(self.intro.tab_2, self.icon2, "Sosyal Medya")

        self.icon3 = QIcon()
        self.icon3.addFile(str(archive_path/"tech.png"))
        self.intro.tabWidget.addTab(self.intro.tab_3, self.icon3, "Robotlar")

        self.icon4 = QIcon()
        self.icon4.addFile(str(archive_path/"gripper.png"))
        self.intro.tabWidget.addTab(self.intro.tab_4, self.icon4, "Robot Kol")

        self.intro.tab1_pic.setPixmap(QPixmap(str(archive_path/"rake.png")))
        self.intro.tab2_pic.setPixmap(QPixmap(str(archive_path/"iturake.jpeg")))
        self.intro.tab3_pic.setPixmap(QPixmap(str(archive_path/"hope.png")))
        self.intro.tab3_pic2.setPixmap(QPixmap(str(archive_path/"dev.png")))
        self.intro.tab3_pic3.setPixmap(QPixmap(str(archive_path/"tika.png")))
        self.intro.tab4_pic.setPixmap(QPixmap(str(archive_path/"robotkol.png")))

        self.intro.pushButton.setFont(QFont("VCR OSD Mono"))
        self.intro.pushButton_2.setFont(QFont("VCR OSD Mono"))
        self.intro.label.setFont(QFont("Helvetica Neue Medium Extended",25))

        #Social Media URL's
        self.intro.tab2_text.setText('<a href="https://rake.itu.edu.tr/">RAKE Resmi Web Sitesi</a>')
        self.intro.tab2_text.setOpenExternalLinks(True)

        self.intro.tab2_text2.setText('<a href="https://www.linkedin.com/company/iturake/mycompany/">LinkedIn</a>')
        self.intro.tab2_text2.setOpenExternalLinks(True)

        self.intro.tab2_text3.setText('<a href="https://www.instagram.com/itu.rake/">Instagram</a>')
        self.intro.tab2_text3.setOpenExternalLinks(True)

        self.intro.tab2_text4.setText('<a href="https://www.facebook.com/rake.itu/">Facebook</a>')
        self.intro.tab2_text4.setOpenExternalLinks(True)

        self.intro.tab2_text5.setText('<a href="https://twitter.com/iturake">Twitter</a>')
        self.intro.tab2_text5.setOpenExternalLinks(True)

        self.intro.tab2_text6.setText('<a href="https://www.youtube.com/@iturake">Youtube</a>')
        self.intro.tab2_text6.setOpenExternalLinks(True)

        self.intro.tab1_text.setWordWrap(True)
        self.intro.tab3_text.adjustSize()
        self.intro.tab3_text.setWordWrap(True)
        self.intro.tab3_text2.setWordWrap(True)
        self.intro.tab3_text3.setWordWrap(True)
        self.intro.tab4_text.setWordWrap(True)
        
        # BUTTON ACTIONS
        self.button1.clicked.connect(self.openWindow_auto)
        self.button2.clicked.connect(self.openWindow_manuel)

        # NODE INITIALIZATION (!!)
        rospy.init_node('RAKE_GUI', anonymous=True)

    # Opening the same window with slight modifications based on autonomous or manuel pick in the intro page.
    def openWindow_auto(self):
        
        qdarktheme.setup_theme()
        
        self.win = window2(self.app) 
        self.win.mode.setText("AUTONOMOUS\n""MODE") #ENG
        
        self.win.mode.setText("OTONOM\n""SÜRÜŞ\n""MODU") #TR
        self.win.resize(width, height)
        self.win.show()
        window1.hide()
        if not QApplication.instance():
            app = QApplication(sys.argv)
        else:
            app = QApplication.instance()

    def openWindow_manuel(self):
        qdarktheme.setup_theme()
        
        self.win = window2(self.app)
        self.win.mode.setText("MANUEL\n""CONTROL\n""MODE") #ENG
        self.win.mode.setText("EL İLE\n""UZAKTAN\n"" KONTROL\n""MODU") #TR
        
        self.win.resize(width, height)
        self.win.show()
        window1.hide()
        if not QApplication.instance():
            app = QApplication(sys.argv)
        else:
            app = QApplication.instance()
        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    
    qdarktheme.setup_theme()
    
    window1 = window1(app)
    
    

    window1.resize(width,height)
    window1.show()
    sys.exit(app.exec_())
