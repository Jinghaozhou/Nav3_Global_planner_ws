#include <array>
#include <memory>
#include <string>

#include <QApplication>
#include <QClipboard>
#include <QColor>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QPainter>
#include <QPixmap>
#include <QPushButton>
#include <QVBoxLayout>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/interaction/view_picker.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace pct_planner_viz
{

class PlaceWaypointTool : public rviz_common::Tool
{
public:
  PlaceWaypointTool()
  {
    role_ = new rviz_common::properties::StringProperty(
      "Waypoint", "start", "start or goal", getPropertyContainer());
    updateIcon();
  }

  void load(const rviz_common::Config & config) override
  {
    rviz_common::Tool::load(config);
    updateIcon();
  }

  void updateIcon()
  {
    const bool goal = role_->getStdString() == "goal";
    QPixmap pixmap(32, 32);
    pixmap.fill(Qt::transparent);
    QPainter painter(&pixmap);
    painter.setPen(Qt::NoPen);
    painter.setBrush(goal ? QColor(242, 150, 79) : QColor(70, 168, 232));
    painter.drawEllipse(3, 3, 26, 26);
    painter.setPen(Qt::white);
    painter.drawText(pixmap.rect(), Qt::AlignCenter, goal ? "G" : "S");
    setIcon(QIcon(pixmap));
  }

  void onInitialize() override
  {
    auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    start_click_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
      "/pct/ui/start_click", 10);
    goal_click_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
      "/pct/ui/goal_click", 10);
    start_reset_ = node->create_publisher<std_msgs::msg::Empty>("/pct/ui/start_reset", 10);
    goal_reset_ = node->create_publisher<std_msgs::msg::Empty>("/pct/ui/goal_reset", 10);
    preview_ = std::make_unique<rviz_rendering::Shape>(
      rviz_rendering::Shape::Sphere, context_->getSceneManager(),
      context_->getSceneManager()->getRootSceneNode());
    preview_->setScale(Ogre::Vector3(0.35f, 0.35f, 0.35f));
    preview_->getRootNode()->setVisible(false);
  }

  void activate() override
  {
    is_goal_ = role_->getStdString() == "goal";
    if (is_goal_) {
      goal_reset_->publish(std_msgs::msg::Empty());
      preview_->setColor(0.95f, 0.59f, 0.31f, 0.85f);
    } else {
      start_reset_->publish(std_msgs::msg::Empty());
      preview_->setColor(0.27f, 0.66f, 0.91f, 0.85f);
    }
    preview_->getRootNode()->setVisible(false);
    setStatus("Move over the 3D map, then left-click to place the waypoint.");
  }

  void deactivate() override
  {
    if (preview_) {
      preview_->getRootNode()->setVisible(false);
    }
  }

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override
  {
    // Hide the preview during depth picking so it cannot pick itself.
    preview_->getRootNode()->setVisible(false);
    if (context_->getFixedFrame() != "map") {
      setStatus("Set RViz Fixed Frame to map before placing waypoints.");
      return Render;
    }
    Ogre::Vector3 hit;
    if (!context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, hit)) {
      setStatus("Move over a visible 3D map surface.");
      return Render;
    }
    preview_->setPosition(hit);
    preview_->getRootNode()->setVisible(true);
    setStatus(QString("map XYZ: %1, %2, %3 — click to place")
      .arg(hit.x, 0, 'f', 3).arg(hit.y, 0, 'f', 3).arg(hit.z, 0, 'f', 3));
    if (event.leftUp()) {
      geometry_msgs::msg::PointStamped point;
      point.header.frame_id = "map";
      point.point.x = hit.x;
      point.point.y = hit.y;
      point.point.z = hit.z;
      if (is_goal_) {
        goal_click_->publish(point);
      } else {
        start_click_->publish(point);
      }
      preview_->getRootNode()->setVisible(false);
      return Finished | Render;
    }
    return Render;
  }

private:
  rviz_common::properties::StringProperty * role_;
  bool is_goal_ = false;
  std::unique_ptr<rviz_rendering::Shape> preview_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr start_click_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_click_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr start_reset_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr goal_reset_;
};

class WaypointPanel : public rviz_common::Panel
{
public:
  explicit WaypointPanel(QWidget * parent = nullptr)
  : rviz_common::Panel(parent)
  {
    auto layout = new QVBoxLayout(this);
    auto add_row = [this, layout](const QString & title, QLineEdit *& edit) {
        auto row = new QHBoxLayout();
        row->addWidget(new QLabel(title));
        edit = new QLineEdit();
        edit->setReadOnly(true);
        edit->setPlaceholderText("Not placed");
        row->addWidget(edit);
        auto copy = new QPushButton("Copy XYZ");
        row->addWidget(copy);
        connect(copy, &QPushButton::clicked, this, [edit]() {
          QApplication::clipboard()->setText(edit->text());
        });
        layout->addLayout(row);
      };
    add_row("START", start_edit_);
    add_row("GOAL", goal_edit_);
    copy_command_ = new QPushButton("Copy /pct/plan command");
    copy_command_->setEnabled(false);
    layout->addWidget(copy_command_);
    connect(copy_command_, &QPushButton::clicked, this, [this]() {copyPlanCommand();});
    layout->addStretch();
  }

  void onInitialize() override
  {
    auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    state_sub_ = node->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/pct/ui/waypoint_state", qos,
      [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr message) {
        auto markers = message->markers;
        QMetaObject::invokeMethod(this, [this, markers]() {
          for (const auto & marker : markers) {
            const bool valid = marker.action == visualization_msgs::msg::Marker::ADD;
            if (marker.ns == "start") {
              start_valid_ = valid;
              if (valid) {
                start_ = {{marker.pose.position.x, marker.pose.position.y, marker.pose.position.z}};
              }
              start_edit_->setText(valid ? format(start_) : QString());
            } else if (marker.ns == "goal") {
              goal_valid_ = valid;
              if (valid) {
                goal_ = {{marker.pose.position.x, marker.pose.position.y, marker.pose.position.z}};
              }
              goal_edit_->setText(valid ? format(goal_) : QString());
            }
          }
          copy_command_->setEnabled(start_valid_ && goal_valid_);
        }, Qt::QueuedConnection);
      });
  }

private:
  static QString format(const std::array<double, 3> & xyz)
  {
    return QString("%1, %2, %3")
      .arg(xyz[0], 0, 'f', 3).arg(xyz[1], 0, 'f', 3).arg(xyz[2], 0, 'f', 3);
  }

  void copyPlanCommand()
  {
    if (!start_valid_ || !goal_valid_) {
      return;
    }
    const auto point_yaml = [](const std::array<double, 3> & xyz) {
        return QString("{header: {frame_id: map}, pose: {position: {x: %1, y: %2, z: %3}, orientation: {w: 1.0}}}")
          .arg(xyz[0], 0, 'f', 3).arg(xyz[1], 0, 'f', 3).arg(xyz[2], 0, 'f', 3);
      };
    const QString command = QString(
      "ros2 service call /pct/plan pct_planner_interfaces/srv/Plan "
      "\"{start: %1, goal: %2}\"")
      .arg(point_yaml(start_)).arg(point_yaml(goal_));
    QApplication::clipboard()->setText(command);
  }

  QLineEdit * start_edit_ = nullptr;
  QLineEdit * goal_edit_ = nullptr;
  QPushButton * copy_command_ = nullptr;
  bool start_valid_ = false;
  bool goal_valid_ = false;
  std::array<double, 3> start_{};
  std::array<double, 3> goal_{};
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr state_sub_;
};

}  // namespace pct_planner_viz

PLUGINLIB_EXPORT_CLASS(pct_planner_viz::PlaceWaypointTool, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(pct_planner_viz::WaypointPanel, rviz_common::Panel)
