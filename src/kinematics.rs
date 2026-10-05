// Copyright 2026 SinfonIA Uniandes <sinfonia@uniandes.edu.co>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Minimal URDF kinematics: joint tree plus forward kinematics.
//!
//! The driver only needs poses of a handful of frames, so this is a tree walk,
//! not a full kinematics library. Joint angles arrive keyed by joint name, as
//! ALMotion reports them.

use crate::{Error, Result};
use std::collections::HashMap;

/// A URDF joint, with the origin of its child link in the parent link.
#[derive(Clone, Debug)]
pub struct Joint {
    pub name: String,
    pub parent: String,
    pub child: String,
    pub origin: crate::domain::Pose3,
    pub axis: crate::domain::Vector3,
    pub kind: JointKind,
    pub mimic: Option<Mimic>,
}

/// Joint degrees of freedom.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum JointKind {
    Fixed,
    Revolute,
    Prismatic,
}

/// A joint driven by another joint, as `<mimic>` expresses it.
#[derive(Clone, Debug)]
pub struct Mimic {
    pub joint: String,
    pub multiplier: f32,
    pub offset: f32,
}

/// The kinematic tree of a robot description.
#[derive(Clone, Debug, Default)]
pub struct RobotModel {
    joints: Vec<Joint>,
    by_name: HashMap<String, usize>,
    by_child_link: HashMap<String, usize>,
}

impl RobotModel {
    /// Parses the `<joint>` elements of a URDF document.
    pub fn from_urdf(xml: &str) -> Result<Self> {
        let mut reader = quick_xml::Reader::from_str(xml);
        reader.config_mut().trim_text(true);
        let mut model = Self::default();
        let mut current: Option<PendingJoint> = None;
        let mut depth = 0usize;
        loop {
            match reader.read_event() {
                Ok(quick_xml::events::Event::Start(element)) => {
                    match (depth, element.name().as_ref()) {
                        (1, b"joint") => current = Some(PendingJoint::new(&element)),
                        (2, _) => {
                            if let Some(joint) = current.as_mut() {
                                joint.read_child(&element);
                            }
                        }
                        _ => {}
                    }
                    depth += 1;
                }
                // Child elements such as `<parent link="..."/>` are empty.
                Ok(quick_xml::events::Event::Empty(element)) if depth == 2 => {
                    if let Some(joint) = current.as_mut() {
                        joint.read_child(&element);
                    }
                }
                Ok(quick_xml::events::Event::End(element)) => {
                    depth = depth.saturating_sub(1);
                    if depth == 1
                        && element.name().as_ref() == b"joint"
                        && let Some(joint) = current.take()
                    {
                        model.add_joint(joint.into_joint()?);
                    }
                }
                Ok(quick_xml::events::Event::Eof) => break,
                Ok(_) => {}
                Err(err) => return Err(Error::Asset(format!("URDF parse error: {err}"))),
            }
        }
        Ok(model)
    }

    fn add_joint(&mut self, joint: Joint) {
        let index = self.joints.len();
        self.by_name.insert(joint.name.clone(), index);
        self.by_child_link.insert(joint.child.clone(), index);
        self.joints.push(joint);
    }

    pub fn joint(&self, name: &str) -> Option<&Joint> {
        self.by_name.get(name).map(|index| &self.joints[*index])
    }

    pub fn joints(&self) -> &[Joint] {
        &self.joints
    }

    /// Pose of `link` in its direct parent link, together with that parent's
    /// name.
    pub fn link_pose_in_parent(
        &self,
        link: &str,
        angles: &HashMap<String, f32>,
    ) -> Option<(String, crate::domain::Pose3)> {
        let joint = &self.joints[*self.by_child_link.get(link)?];
        Some((joint.parent.clone(), self.joint_transform(joint, angles)))
    }

    /// Pose of `link` expressed in `root`, given joint positions by name.
    ///
    /// Returns `None` when the links are not connected or an involved joint has
    /// no known angle.
    pub fn link_pose(
        &self,
        root: &str,
        link: &str,
        angles: &HashMap<String, f32>,
    ) -> Option<crate::domain::Pose3> {
        let mut joints = Vec::new();
        let mut cursor = link;
        while cursor != root {
            let index = *self.by_child_link.get(cursor)?;
            joints.push(&self.joints[index]);
            cursor = &self.joints[index].parent;
        }
        let mut pose = crate::domain::Pose3::IDENTITY;
        for joint in joints.iter().rev() {
            pose = pose.compose(self.joint_transform(joint, angles));
        }
        Some(pose)
    }

    fn joint_transform(
        &self,
        joint: &Joint,
        angles: &HashMap<String, f32>,
    ) -> crate::domain::Pose3 {
        let displacement = self.joint_displacement(joint, angles);
        let motion = match joint.kind {
            JointKind::Fixed => crate::domain::Pose3::IDENTITY,
            JointKind::Revolute => crate::domain::Pose3 {
                position: crate::domain::Vector3::new(0.0, 0.0, 0.0),
                orientation: crate::domain::Quaternion::from_axis_angle(joint.axis, displacement),
            },
            JointKind::Prismatic => crate::domain::Pose3 {
                position: joint.axis * displacement,
                orientation: crate::domain::Quaternion::IDENTITY,
            },
        };
        crate::domain::Pose3 {
            position: joint.origin.position,
            orientation: joint.origin.orientation,
        }
        .compose(motion)
    }

    fn joint_displacement(&self, joint: &Joint, angles: &HashMap<String, f32>) -> f32 {
        match &joint.mimic {
            Some(mimic) => {
                self.joint_displacement_named(&mimic.joint, angles) * mimic.multiplier
                    + mimic.offset
            }
            None => angles.get(&joint.name).copied().unwrap_or(0.0),
        }
    }

    fn joint_displacement_named(&self, name: &str, angles: &HashMap<String, f32>) -> f32 {
        match self.joint(name) {
            Some(joint) => self.joint_displacement(joint, angles),
            None => 0.0,
        }
    }
}

/// Accumulates one `<joint>` element while streaming the document.
struct PendingJoint {
    name: String,
    kind: String,
    parent: Option<String>,
    child: Option<String>,
    origin: crate::domain::Pose3,
    axis: crate::domain::Vector3,
    mimic: Option<Mimic>,
}

impl PendingJoint {
    fn new(element: &quick_xml::events::BytesStart<'_>) -> Self {
        let mut joint = Self {
            name: String::new(),
            kind: String::new(),
            parent: None,
            child: None,
            origin: crate::domain::Pose3::IDENTITY,
            axis: crate::domain::Vector3::new(0.0, 0.0, 1.0),
            mimic: None,
        };
        for attribute in element.attributes().flatten() {
            match attribute.key.as_ref() {
                b"name" => joint.name = attribute_value(&attribute),
                b"type" => joint.kind = attribute_value(&attribute),
                _ => {}
            }
        }
        joint
    }

    fn read_child(&mut self, element: &quick_xml::events::BytesStart<'_>) {
        let attribute = |name: &[u8]| {
            element
                .attributes()
                .flatten()
                .find(|attribute| attribute.key.as_ref() == name)
                .map(|attribute| attribute_value(&attribute))
        };
        match element.name().as_ref() {
            b"parent" => self.parent = attribute(b"link"),
            b"child" => self.child = attribute(b"link"),
            b"origin" => {
                self.origin =
                    parse_pose(attribute(b"xyz").as_deref(), attribute(b"rpy").as_deref());
            }
            b"axis" => {
                self.axis = parse_vector(attribute(b"xyz").as_deref())
                    .unwrap_or(crate::domain::Vector3::new(0.0, 0.0, 1.0));
            }
            b"mimic" => {
                self.mimic = attribute(b"joint").map(|source| Mimic {
                    joint: source,
                    multiplier: attribute(b"multiplier")
                        .and_then(|value| value.parse().ok())
                        .unwrap_or(1.0),
                    offset: attribute(b"offset")
                        .and_then(|value| value.parse().ok())
                        .unwrap_or(0.0),
                });
            }
            _ => {}
        }
    }

    fn into_joint(self) -> Result<Joint> {
        let kind = match self.kind.as_str() {
            "fixed" => JointKind::Fixed,
            "revolute" | "continuous" => JointKind::Revolute,
            "prismatic" => JointKind::Prismatic,
            other => return Err(Error::Asset(format!("unsupported URDF joint type {other}"))),
        };
        let name = self.name;
        let parent = self
            .parent
            .ok_or_else(|| Error::Asset(format!("joint {name} misses its parent")))?;
        let child = self
            .child
            .ok_or_else(|| Error::Asset(format!("joint {name} misses its child")))?;
        Ok(Joint {
            name,
            parent,
            child,
            origin: self.origin,
            axis: self.axis,
            kind,
            mimic: self.mimic,
        })
    }
}

fn attribute_value(attribute: &quick_xml::events::attributes::Attribute<'_>) -> String {
    String::from_utf8_lossy(&attribute.value).into_owned()
}

fn parse_vector(xyz: Option<&str>) -> Option<crate::domain::Vector3> {
    let mut components = xyz?.split_whitespace().map(|value| value.parse().ok());
    Some(crate::domain::Vector3::new(
        components.next().flatten()?,
        components.next().flatten()?,
        components.next().flatten()?,
    ))
}

fn parse_pose(xyz: Option<&str>, rpy: Option<&str>) -> crate::domain::Pose3 {
    let [x, y, z] = vector3(xyz);
    let [roll, pitch, yaw] = vector3(rpy);
    crate::domain::Pose3 {
        position: crate::domain::Vector3::new(x, y, z),
        orientation: crate::domain::Quaternion::from_euler(roll, pitch, yaw),
    }
}

fn vector3(value: Option<&str>) -> [f32; 3] {
    let mut result = [0.0; 3];
    if let Some(value) = value {
        for (slot, part) in result.iter_mut().zip(value.split_whitespace()) {
            if let Ok(number) = part.parse() {
                *slot = number;
            }
        }
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    const PEPPER_URDF: &str = include_str!("../share/urdf/pepper.urdf");

    fn model() -> RobotModel {
        RobotModel::from_urdf(PEPPER_URDF).expect("valid URDF")
    }

    #[test]
    fn parses_pepper_joints_and_links() {
        let model = model();
        assert!(model.joint("HeadYaw").is_some());
        assert_eq!(model.joint("HeadYaw").expect("joint").child, "Neck");
        assert!(model.joints().len() > 50);
    }

    #[test]
    fn fixed_chains_compose_to_the_urdf_origin() {
        let model = model();
        let angles = HashMap::new();
        let pose = model
            .link_pose("Head", "CameraTop_frame", &angles)
            .expect("chain");
        assert!((pose.position.x - 0.0868).abs() < 1e-4);
        assert!((pose.position.z - 0.1631).abs() < 1e-4);
    }

    #[test]
    fn revolute_joints_rotate_their_children() {
        let model = model();
        let mut angles = HashMap::new();
        angles.insert("HeadYaw".to_owned(), std::f32::consts::FRAC_PI_2);
        let pose = model.link_pose("torso", "Neck", &angles).expect("chain");
        // The URDF origin of Neck in torso is (-0.038, 0, 0.1699); yaw rotates
        // the offset of children, not the joint origin itself.
        assert!((pose.position.x - -0.038).abs() < 1e-4);
        let rotated = pose
            .orientation
            .rotate(crate::domain::Vector3::new(1.0, 0.0, 0.0));
        assert!(rotated.y > 0.9);
    }

    #[test]
    fn mimic_joints_follow_their_source() {
        let model = model();
        let mimic = model
            .joints()
            .iter()
            .find(|joint| joint.mimic.is_some())
            .expect("mimic joint");
        let source = mimic.mimic.as_ref().expect("mimic").joint.clone();
        let mut angles = HashMap::new();
        angles.insert(source, 1.0);
        let displacement = model.joint_displacement(mimic, &angles);
        let mimic_spec = mimic.mimic.as_ref().expect("mimic");
        assert!((displacement - mimic_spec.multiplier - mimic_spec.offset).abs() < 1e-5);
    }

    #[test]
    fn disconnected_links_yield_none() {
        let model = model();
        assert!(
            model
                .link_pose("torso", "no_such_link", &HashMap::new())
                .is_none()
        );
    }

    #[test]
    fn link_pose_in_parent_reports_the_direct_parent() {
        let model = model();
        let (parent, pose) = model
            .link_pose_in_parent("Neck", &HashMap::new())
            .expect("joint");
        assert_eq!(parent, "torso");
        assert!((pose.position.z - 0.1699).abs() < 1e-4);
        let (parent, _) = model
            .link_pose_in_parent("CameraTop_frame", &HashMap::new())
            .expect("joint");
        assert_eq!(parent, "Head");
    }

    #[test]
    fn rejects_unparsable_urdf() {
        assert!(RobotModel::from_urdf("<robot><joint").is_err());
    }
}
