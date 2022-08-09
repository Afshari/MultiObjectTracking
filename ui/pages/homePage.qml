import QtQuick 2.12
import QtQuick.Controls 2.12
import "../controls"
import QtQuick.Layouts 1.12

Item {
    Rectangle {
        id: rectangle
        color: "#2c313c"
        anchors.fill: parent

        Rectangle {
            id: rectangleTop
            height: 69
            color: "#495163"
            radius: 10
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.rightMargin: 50
            anchors.leftMargin: 50
            anchors.topMargin: 40

            //GridLayout {
            //    anchors.fill: parent
            //    anchors.rightMargin: 10
            //    anchors.leftMargin: 10
            //    rows: 1
            //    columns: 3

            //    CustomTextField{
            //        id: textField
            //        placeholderText: "Type your name"
            //        Layout.fillWidth: true
            //        Keys.onEnterPressed: {
            //            backend.welcomeText(textField.text)
            //        }
            //        Keys.onReturnPressed: {
            //            backend.welcomeText(textField.text)
            //        }
            //    }

            //    CustomButton{
            //        id: btnChangeName
            //        text: "Change Name"
            //        Layout.maximumWidth: 200
            //        Layout.fillWidth: true
            //        Layout.preferredHeight: 40
            //        Layout.preferredWidth: 250
            //        onClicked: {
            //            backend.welcomeText(textField.text)
            //        }
            //    }

            //    Switch {
            //        id: switchHome
            //        text: qsTr("Switch")
            //        checked: true
            //        Layout.preferredHeight: 40
            //        Layout.preferredWidth: 68
            //        // Change Show/Hide Frame
            //        onToggled: {
            //            backend.showHideRectangle(switchHome.checked)
            //        }
            //    }
            //}
        }

        Rectangle {
            id: rectangleVisible
            color: "#1d2128"
            radius: 10
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.top: rectangleTop.bottom
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 40
            anchors.rightMargin: 50
            anchors.leftMargin: 50
            anchors.topMargin: 10

            Label {
                id: labelTextName
                y: 8
                height: 25
                color: "#5c667d"
                text: qsTr("Multi Object Tracking in Clutter")
                anchors.left: parent.left
                anchors.right: parent.right
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                anchors.leftMargin: 10
                anchors.rightMargin: 10
                font.pointSize: 14
            }

            Label {
                id: labelDate
                y: 31
                height: 25
                color: "#55aaff"
                text: qsTr("")
                anchors.left: parent.left
                anchors.right: parent.right
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                anchors.rightMargin: 10
                anchors.leftMargin: 10
                font.pointSize: 12
            }

            ScrollView {
                id: scrollView
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: labelDate.bottom
                anchors.bottom: parent.bottom
                clip: true
                anchors.rightMargin: 10
                anchors.leftMargin: 10
                anchors.bottomMargin: 10
                anchors.topMargin: 10

                Text {
                    id: textHome
                    color: "#a9b2c8"
                    text: "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n<html><head></head><body><h2>This Project is about Multi Object Tracking in Clutter</h2><h3>*Single Object Tracking Algorithms</h3><ul><li>Nearest Neighbor</li><li>Probabilistic Data Association</li><li>Gaussian Sum</li></ul><h3>*Multi Object Tracking Algorithms</h3><ul><li>Global Nearest Neighbor</li><li>Joint Probabilistic Data Association</li><li>Multi Hypothesis Tracking</li></ul></body></html>"
                    anchors.fill: parent
                    font.pixelSize: 12
                    textFormat: Text.RichText
                }
            }
        }
    }
}

/*##^##
Designer {
    D{i:0;autoSize:true;height:480;width:800}
}
##^##*/
