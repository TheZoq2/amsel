
use scad::*;
use scad_util::constants::{x_axis, y_axis};

use std::path::{PathBuf};

mod wing;
mod hinge;

use crate::wing::{Airfoil, Wing};

qstruct!(Fuselage () {
    x_length: f32 = 400.,
    y_width: f32 = 40.,
    z_height: f32 = 40.,
    back_y_width: f32 = 22.,
    wing_start: f32 = 150.
});

impl Fuselage {
    pub fn get(&self) -> ScadObject {
        let extrude_params = LinExtrudeParams {
            height: 1000.,
            center: true,
            .. Default::default()
        };

        let xy_shape = scad!(LinearExtrude(extrude_params.clone()); {
            self.xy_outline()
        });
        let xz_shape = scad!(LinearExtrude(extrude_params.clone()); {
            self.xz_outline()
        });

        scad!(Intersection; {
            xy_shape,
            scad!(Rotate(90., x_axis()); xz_shape)
        })
    }

    pub fn xy_outline(&self) -> ScadObject {
        let points = vec!(
                vec2(0., self.back_y_width / 2.),
                vec2(self.wing_start, self.y_width / 2.),
                vec2(self.x_length, self.y_width / 2.),
                vec2(self.x_length, -self.y_width / 2.),
                vec2(self.wing_start, -self.y_width / 2.),
                vec2(0., -self.back_y_width / 2.),
            );

        scad!(Polygon(PolygonParameters::new(points)))
    }

    pub fn xz_outline(&self) -> ScadObject {
        let oval_radius = 100.;
        let front_x = self.x_length - oval_radius;
        let points = vec!(
                vec2(0., self.back_y_width / 2.),
                vec2(self.wing_start, self.y_width / 2.),
                vec2(front_x, self.y_width / 2.),
                vec2(front_x, -self.y_width / 2.),
                vec2(self.wing_start, -self.y_width / 2.),
                vec2(0., -self.back_y_width / 2.),
            );

        let oval = {
            let shape = scad!(Scale2d(vec2(oval_radius, self.z_height / 2.)); {
                scad!(Circle(Radius(1.)))
            });
            scad!(Translate2d(vec2(front_x, 0.)); shape)
        };

        scad!(Union; {
            scad!(Polygon(PolygonParameters::new(points))),
            oval,
        })
    }
}

fn extrude_airfoil(airfoil: &Airfoil, length: f32, height: f32) -> ScadObject {
    let shape = scad!(Polygon(PolygonParameters::new(airfoil.points.clone())));

    scad!(LinearExtrude(LinExtrudeParams{ height, .. Default::default()}); {
        scad!(Scale2d(vec2(length, length)); shape)
    })
}

fn wing() -> Wing {
    let airfoil = Airfoil::load_from_file(&PathBuf::from("airfoils/mh44.dat"));
    Wing::new(airfoil)
}


fn assemble_plane() -> ScadObject {
    let wing_object = wing();

    let wing = scad!(Rotate(90., x_axis()); {
        scad!(Mirror(x_axis()); {
            scad!(Translate(-x_axis() * wing_object.inner_length); {
                wing_object.extruded_rib_shape()
            })
        })
    });

    let fuselage = Fuselage::new().get();

    scad!(Union; {
        wing.clone(),
        scad!(Mirror(y_axis()); wing.clone()),
        fuselage
    })
}

fn wing_ribs(range: std::ops::Range<usize>) -> ScadObject {
    // wing().all_wing_ribs(5., 8)
    let wing = wing();
    let count = 8;
    let thickness = 5.;
    let mut result = scad!(Union);
    for i in range {
        result.add_child(scad!(Translate(vec3(0., 15. * i as f32, 0.)); {
            wing.get_wing_rib(i, thickness, count)
        }));
    }
    result
}



fn main() {
    let airfoil = Airfoil::load_from_file(&PathBuf::from("airfoils/mh44.dat"));

    let wing = Wing::new(airfoil.clone());

    let mut file = ScadFile::new();
    file.set_detail(25);
    file.add_object(extrude_airfoil(&airfoil, 140., 10.));
    // file.add_object(assemble_plane());
    // file.add_object(wing_ribs(0..3));
    // file.add_object(wing);
    // file.add_object(wing.control_surface(0.45, 30.));
    // file.add_object(hinge::hinge(5, 4., 4.));
    // file.add_object(hinge::test_hinge());
    file.write_to_file("out.scad".into());
}
