import QtQuick 2.12
import QtQml 2.12
import QtQuick.Controls 2.12
import QtGraphicalEffects 1.0


Button {

    id: btnToggle
    // CUSTOM PROPERTIES
    property url   btnIconSource: "../../ui/images/svg_images/menu_icon.svg"
    property color btnColorDefault: "#1c1d20"
    property color btnColorMouseOver: "#23272E"
    property color btnColorClicked: "#00a1f1"
    property string btnText: "mohsen"

    QtObject {
        id: internal

        // MOUSE OVER AND CLICK CHANGE COLOR
        property var dynamicColor: if(btnToggle.down) {
                                       btnToggle.down ? btnColorClicked : btnColorDefault
                                   } else {
                                       btnToggle.hovered ? btnColorMouseOver : btnColorDefault
                                   }
    }

    implicitWidth:  70
    implicitHeight: 80


    background: Rectangle {
        id: bgBtn
        color: internal.dynamicColor

        Image {
            id: iconBtn
            source: btnIconSource
            // anchors.verticalCenter:     parent.verticalCenter
            anchors.top: parent.top
            height: 25
            width: 25
            anchors.topMargin: (btnToggle.height-height-10)/2
            anchors.horizontalCenter:   parent.horizontalCenter
            fillMode: Image.PreserveAspectFit
            visible: false
        }

        ColorOverlay {
            anchors.fill: iconBtn
            source: iconBtn
            color: "#ffffff"
            antialiasing: false
        }
    }

    contentItem: Item {
        anchors.fill: parent
        id: content

        Text {
            color: "#ffffff"
            text: btnToggle.btnText
            // font: btnLeftMenu.font
             anchors.horizontalCenter: parent.horizontalCenter
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 5
        }
    }
}





