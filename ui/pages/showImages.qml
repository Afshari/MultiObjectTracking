import QtQuick 2.12
import QtQuick.Controls 2.12

Item {
    Rectangle {
        id: rectangle
        color: "#2c313c"
        anchors.fill: parent

        Image {

            anchors {
                horizontalCenter: parent.horizontalCenter
                verticalCenter: parent.verticalCenter
            }

            width: 300
            height: 300
//            source: "file:///Users/mohsen/Desktop/Paas/Julia/2127547_3a2918_901x1200_v0.jpg"
        }

//        Label {
//            id: label
//            x: 302
//            y: 194
//            color: "#ffffff"
//            text: qsTr("Show Images")
//            anchors.verticalCenter: parent.verticalCenter
//            horizontalAlignment: Text.AlignHCenter
//            verticalAlignment: Text.AlignVCenter
//            anchors.horizontalCenter: parent.horizontalCenter
//            font.pointSize: 16
//        }
    }

}

/*##^##
Designer {
    D{i:0;autoSize:true;height:480;width:800}
}
##^##*/
