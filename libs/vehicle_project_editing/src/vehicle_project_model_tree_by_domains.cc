#include "orvd/vehicle_project_editing/vehicle_project_model_tree_by_domains.h"

#include <algorithm>
#include <cstddef>
#include <map>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "orvd/vehicle_project/vehicle_project_element_kind_records.h"
#include "vehicle_project_element_name_spaces.h"

namespace orvd::vehicle_project_editing {
namespace {

using vehicle_project::ElementKind;
using vehicle_project::ElementReference;
using vehicle_project::VehicleProject;

// Row keys. A row of its own, a domain or settings, has a word without ':'
// for its key. An element row's key is its kind's word, followed for a
// record repeating a kind and name already listed by '#' and the record's
// count among them, then ':' and the element's name; a row below an element
// row has what it is, ':' and the element row's key. What lies before the
// first ':' says which row it is and what follows whose, so no two rows share
// a key, while a project file holding two records of one kind under one name
// still has a row for each.
class ElementRowKeys {
  public:
    std::string KeyOf(const ElementReference& element) {
        const std::size_t count = ++listed_[{element.kind, element.name}];
        std::string key(vehicle_project::ElementKindWord(element.kind));
        if (count > 1) {
            key += "#" + std::to_string(count);
        }
        return key + ":" + element.name;
    }

  private:
    std::map<std::pair<ElementKind, std::string>, std::size_t> listed_;
};

ModelTreeNode ElementRow(ElementRowKeys& keys, const ElementReference& element) {
    return {.key = keys.KeyOf(element), .subject = element};
}

ModelTreeNode SettingsRow(ProjectSettings settings, Title title,
                          std::optional<Title> note = std::nullopt) {
    return {.key = std::string(ProjectSettingsWord(settings)),
            .title = std::move(title),
            .subject = settings,
            .note = std::move(note)};
}

ModelTreeNode NotProvidedRow(std::string key, Title title, Title note) {
    return {.key = std::move(key),
            .title = std::move(title),
            .provided = false,
            .note = std::move(note)};
}

ModelTreeNode DescribeGlobals() {
    return {
        .key = "globals",
        .title = Title{.en = "Globals", .zh = "全局"},
        .children = {
            SettingsRow(ProjectSettings::kGravity, {.en = "Gravity", .zh = "重力加速度"}),
            SettingsRow(ProjectSettings::kVehicleIdentity,
                        {.en = "Vehicle identity", .zh = "车辆标识"},
                        Title{.en = "Save as gives the vehicle both names.",
                              .zh = "另存为时设定这两个名称。"}),
            SettingsRow(ProjectSettings::kStationLayout, {.en = "Station layout", .zh = "站位布局"},
                        Title{.en = "The body the stations of the free bodies are measured "
                                    "from.",
                              .zh = "各自由体站位的度量基准刚体。"}),
            NotProvidedRow("initial_conditions", {.en = "Initial conditions", .zh = "初始条件"},
                           {.en = "The state a run starts from: the bodies' initial "
                                  "positions and velocities.",
                            .zh = "运行开始时的状态：各刚体的初始位置与速度。"}),
            NotProvidedRow(
                "views", {.en = "Views", .zh = "观察视角"},
                {.en = "Saved viewpoints of the 3D view.", .zh = "三维视图的保存视角。"}),
        }};
}

// The name a body's shapes are stated under, or that none are.
Title DescribeGeometry(const VehicleProject& project, const std::string& body_name) {
    const auto named = std::ranges::find(project.visual_definition_body_names, body_name,
                                         &vehicle_project::VisualDefinitionBodyName::body_name);
    if (named == project.visual_definition_body_names.end()) {
        return {.en = "No shapes are stated for it.", .zh = "未记述它的形状。"};
    }
    return {.en = "Its shapes are stated under '" + named->visual_definition_body_name +
                  "' in the visual definition.",
            .zh = "其形状在显示定义中以“" + named->visual_definition_body_name + "”记述。"};
}

// Every rigid body with its markers and geometry, then the markers whose body
// the project does not have. A marker is listed under the first body of its
// body's name.
ModelTreeNode DescribeBodies(const VehicleProject& project, ElementRowKeys& keys) {
    ModelTreeNode bodies{.key = "bodies",
                         .title = Title{.en = "Bodies", .zh = "刚体"},
                         .creates = {{.kind = ElementKind::kRigidBody}}};
    const auto& frames = project.vehicle_definition.fixed_frames;
    std::vector<bool> listed(frames.size(), false);
    for (const configuration::VehicleRigidBodyDefinition& body :
         project.vehicle_definition.rigid_bodies) {
        ModelTreeNode body_row = ElementRow(keys, {ElementKind::kRigidBody, body.name});
        ModelTreeNode markers{
            .key = "markers:" + body_row.key,
            .title = Title{.en = "Markers", .zh = "标记点"},
            .creates = {{.kind = ElementKind::kFixedFrame,
                         .fixed_properties = {{"body_name", ReferencedElementName{body.name}}}}}};
        for (std::size_t index = 0; index < frames.size(); ++index) {
            if (!listed[index] && frames[index].body_name == body.name) {
                markers.children.push_back(
                    ElementRow(keys, {ElementKind::kFixedFrame, frames[index].name}));
                listed[index] = true;
            }
        }
        body_row.children.push_back(std::move(markers));
        body_row.children.push_back({.key = "geometry:" + body_row.key,
                                     .title = Title{.en = "Geometry", .zh = "几何"},
                                     .note = DescribeGeometry(project, body.name)});
        bodies.children.push_back(std::move(body_row));
    }
    ModelTreeNode owner_not_found{
        .key = "markers_owner_not_found",
        .title = Title{.en = "Owner not found", .zh = "归属未解析"},
        .note = Title{.en = "Markers naming a body this project does not have.",
                      .zh = "所指刚体不在本工程中的标记点。"}};
    for (std::size_t index = 0; index < frames.size(); ++index) {
        if (!listed[index]) {
            owner_not_found.children.push_back(
                ElementRow(keys, {ElementKind::kFixedFrame, frames[index].name}));
        }
    }
    if (!owner_not_found.children.empty()) {
        bodies.children.push_back(std::move(owner_not_found));
    }
    return bodies;
}

// One list of the elements of `kinds`, in outline order, creating any of them.
ModelTreeNode DescribeElementList(const VehicleProject& project, ElementRowKeys& keys,
                                  std::string key, Title title,
                                  std::span<const ElementKind> kinds) {
    ModelTreeNode list{.key = std::move(key), .title = std::move(title)};
    for (const ElementKind kind : kinds) {
        list.creates.push_back({.kind = kind});
        for (const std::string& name : CollectElementNames(project, kind)) {
            list.children.push_back(ElementRow(keys, {kind, name}));
        }
    }
    return list;
}

// The contact carriers, each with the interfaces naming it, then the
// interfaces whose carrier the project does not have. An interface is listed
// under the first carrier of its carrier's name.
ModelTreeNode DescribeContactCarriers(const configuration::VehicleContactDeclaration& declaration,
                                      ElementRowKeys& keys) {
    ModelTreeNode carriers{
        .key = "contact_carriers",
        .title = Title{.en = "Contact carriers", .zh = "接触承载体"},
        .note = Title{.en = "The bodies carrying the wheel-rail contacts: rigid wheelsets, or axle "
                            "bridges whose wheels turn independently.",
                      .zh = "承担轮轨接触的刚体：刚性轮对，或车轮独立旋转的轴桥。"},
        .creates = {{.kind = ElementKind::kContactCarrier}}};
    const auto& interfaces = declaration.interfaces;
    std::vector<bool> listed(interfaces.size(), false);
    for (const configuration::VehicleContactCarrierDeclaration& carrier : declaration.carriers) {
        ModelTreeNode carrier_row =
            ElementRow(keys, {ElementKind::kContactCarrier, carrier.carrier_name});
        carrier_row.creates.push_back(
            {.kind = ElementKind::kContactInterface,
             .fixed_properties = {{"carrier_name", ReferencedElementName{carrier.carrier_name}}}});
        for (std::size_t index = 0; index < interfaces.size(); ++index) {
            if (!listed[index] && interfaces[index].carrier_name == carrier.carrier_name) {
                carrier_row.children.push_back(ElementRow(
                    keys, {ElementKind::kContactInterface, interfaces[index].interface_name}));
                listed[index] = true;
            }
        }
        carriers.children.push_back(std::move(carrier_row));
    }
    ModelTreeNode owner_not_found{
        .key = "contact_interfaces_owner_not_found",
        .title = Title{.en = "Owner not found", .zh = "归属未解析"},
        .note = Title{.en = "Wheel-rail contacts naming a carrier this project does not have.",
                      .zh = "所指承载体不在本工程中的轮轨接触。"}};
    for (std::size_t index = 0; index < interfaces.size(); ++index) {
        if (!listed[index]) {
            owner_not_found.children.push_back(ElementRow(
                keys, {ElementKind::kContactInterface, interfaces[index].interface_name}));
        }
    }
    if (!owner_not_found.children.empty()) {
        carriers.children.push_back(std::move(owner_not_found));
    }
    return carriers;
}

ModelTreeNode DescribeWheelRail(const VehicleProject& project, ElementRowKeys& keys) {
    ModelTreeNode wheel_rail{.key = "wheel_rail", .title = Title{.en = "Wheel-rail", .zh = "轮轨"}};
    if (!project.contact_record.has_value()) {
        wheel_rail.note = Title{.en = "The project has no wheel-rail contact record yet.",
                                .zh = "本工程尚无轮轨接触记录。"};
        return wheel_rail;
    }
    wheel_rail.children.push_back(
        SettingsRow(ProjectSettings::kWheel, {.en = "Wheel", .zh = "车轮"}));
    wheel_rail.children.push_back(
        SettingsRow(ProjectSettings::kRail, {.en = "Rail", .zh = "钢轨"}));
    wheel_rail.children.push_back(SettingsRow(ProjectSettings::kContactComputation,
                                              {.en = "Contact computation", .zh = "接触计算"}));
    wheel_rail.children.push_back(
        DescribeContactCarriers(project.contact_record->declaration, keys));
    return wheel_rail;
}

}  // namespace

std::vector<ModelTreeNode> DescribeModelTree(const VehicleProject& project) {
    ElementRowKeys keys;
    std::vector<ModelTreeNode> domains;
    domains.push_back(DescribeGlobals());
    domains.push_back(DescribeBodies(project, keys));
    domains.push_back(DescribeElementList(project, keys, "joints", {.en = "Joints", .zh = "铰"},
                                          internal::kJointKinds));
    domains.push_back(DescribeElementList(project, keys, "force_elements",
                                          {.en = "Force elements", .zh = "力元"},
                                          internal::kForceElementKinds));
    domains.push_back(NotProvidedRow(
        "constraints", {.en = "Constraints", .zh = "约束"},
        {.en = "Constraints that close kinematic loops.", .zh = "闭合运动链的约束。"}));
    domains.push_back(DescribeWheelRail(project, keys));
    domains.push_back(
        NotProvidedRow("track", {.en = "Track", .zh = "线路"},
                       {.en = "The line: straights, curves, vertical curves, transitions and track "
                              "irregularities.",
                        .zh = "线路：直线、曲线、竖曲线、缓和曲线与轨道不平顺。"}));
    domains.push_back(NotProvidedRow("substructures", {.en = "Substructures", .zh = "子结构"},
                                     {.en = "Other models included in this one, nested to any "
                                            "depth.",
                                      .zh = "引入本模型的其他模型，可逐层嵌套。"}));
    return domains;
}

Title ElementKindTitle(ElementKind kind) {
    switch (kind) {
        case ElementKind::kRigidBody:
            return {.en = "Rigid body", .zh = "刚体"};
        case ElementKind::kFixedFrame:
            return {.en = "Marker", .zh = "标记点"};
        case ElementKind::kRevoluteJoint:
            return {.en = "Revolute joint", .zh = "转动铰"};
        case ElementKind::kBallRpyJoint:
            return {.en = "Ball joint", .zh = "球铰"};
        case ElementKind::kWeldJoint:
            return {.en = "Weld joint", .zh = "固结铰"};
        case ElementKind::kTranslationalSpringDamper:
            return {.en = "Translational spring-damper", .zh = "平动弹簧阻尼"};
        case ElementKind::kRollSpringDamperCouple:
            return {.en = "Roll spring-damper couple", .zh = "侧滚弹簧阻尼力偶"};
        case ElementKind::kSeriesSpringViscousDamper:
            return {.en = "Series spring and viscous damper", .zh = "串联弹簧粘滞阻尼"};
        case ElementKind::kPointToPointSeriesSpringBilinearDamper:
            return {.en = "Point-to-point series spring and bilinear damper",
                    .zh = "点对点串联弹簧双线性阻尼"};
        case ElementKind::kSaturatedPiecewiseLinearDamper:
            return {.en = "Saturated piecewise-linear damper", .zh = "饱和分段线性阻尼"};
        case ElementKind::kReferenceFrameLinearBushing:
            return {.en = "Linear bushing in the reference frame", .zh = "参考坐标系线性衬套"};
        case ElementKind::kPointToPointLinearSpringDamper:
            return {.en = "Point-to-point linear spring-damper", .zh = "点对点线性弹簧阻尼"};
        case ElementKind::kPointToPointHystereticSpring:
            return {.en = "Point-to-point hysteretic spring", .zh = "点对点迟滞弹簧"};
        case ElementKind::kContactCarrier:
            return {.en = "Contact carrier", .zh = "接触承载体"};
        case ElementKind::kContactInterface:
            return {.en = "Wheel-rail contact", .zh = "轮轨接触"};
    }
    throw std::invalid_argument("vehicle project element kind " +
                                std::to_string(std::to_underlying(kind)) +
                                " is none of the element kinds");
}

}  // namespace orvd::vehicle_project_editing
