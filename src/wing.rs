use scad::*;
use scad_util::constants::y_axis;

use nalgebra as na;

use std::path::{Path};
use std::fs;
use std::io::prelude::*;


#[derive(Clone)]
pub struct Airfoil {
    pub points: Vec<na::Vector2<f32>>
}

impl Airfoil {
    pub fn load_from_file(path: &Path) -> Self {
        let mut file = fs::File::open(path).unwrap();
        let mut content = String::new();
        file.read_to_string(&mut content).unwrap();

        let mut lines = content.lines();
        let _first_line = lines.next().expect("No first line in the file");

        let points = lines.map(|line| {
            let mut words = line.split_whitespace();
            let x = words.next().expect("no x").parse::<f32>().expect("could not parse x");
            let y = words.next().expect("no y").parse::<f32>().expect("could not parse y");
            vec2(x, y)
        }).collect::<Vec<_>>();

        Airfoil {
            points
        }
    }
}

qstruct!(Wing (airfoil: Airfoil) {
    inner_length: f32 = 140.,
    airfoil: Airfoil = airfoil,
    outer_length: f32 = 100.,
    wingspan: f32 = 500.,
    extrude_thickness: f32 = 0.1,
    spar_radius: f32 = 5.
});

#[derive(PartialEq)]
enum CutoffDirection {
    Up,
    Down,
    Both
}

impl Wing {
    pub fn exterior_model(&self) -> ScadObject {
        let extrude_function = |obj| {
            let params = LinExtrudeParams {
                height: self.extrude_thickness,
                .. Default::default()
            };
            scad!(LinearExtrude(params); obj)
        };

        let inner = self.exterior_inner_shape();
        let outer = self.exterior_outer_shape();

        scad!(Hull; {
            extrude_function(inner),
            scad!(Translate(vec3(0., 0., self.wingspan - self.extrude_thickness)); {
                extrude_function(outer)
            })
        })
    }

    fn wing_shaped_cutoff_box(
        &self,
        dir: CutoffDirection,
        inner_start: f32,
        inner_length: f32,
        outer_start: f32,
        outer_length: f32
    ) -> ScadObject {
        let y_size = 1000.;
        let center_y = dir == CutoffDirection::Both;
        let translation_y = if dir == CutoffDirection::Down {-y_size} else {0.};
        let inner_block = scad!(Translate(vec3(inner_start, translation_y, 0.)); {
            centered_cube(vec3(inner_length, y_size, 0.001), (false, center_y, false))
        });
        let outer_block = scad!(Translate(vec3(outer_start, translation_y, self.wingspan)); {
            centered_cube(vec3(outer_length, y_size, 0.001), (false, center_y, false))
        });
        scad!(Hull; {
            inner_block,
            outer_block,
        })
    }

    pub fn extruded_rib_shape(&self) -> ScadObject {
        let foam_shape_offset = -2.;

        let foam_front_stop = 10.;
        let foam_upper_back_stop = 40.;
        let foam_lower_back_stop = 35.;
        let foam_inner_upper_cutout_length =
            self.inner_length - foam_front_stop - foam_upper_back_stop;
        let foam_outer_upper_cutout_length =
            self.outer_length - foam_front_stop - foam_upper_back_stop;
        let foam_inner_lower_cutout_length =
            self.inner_length - foam_front_stop - foam_lower_back_stop;
        let foam_outer_lower_cutout_length =
            self.outer_length - foam_front_stop - foam_lower_back_stop;

        let foam_upper_cutout = self.wing_shaped_cutoff_box(
            CutoffDirection::Up,
            foam_front_stop,
            foam_inner_upper_cutout_length,
            foam_front_stop,
            foam_outer_upper_cutout_length
        );
        let foam_lower_cutout = self.wing_shaped_cutoff_box(
            CutoffDirection::Down,
            foam_front_stop,
            foam_inner_lower_cutout_length,
            foam_front_stop,
            foam_outer_lower_cutout_length
        );

        let front_and_back = scad!(Difference; {
            self.exterior_model(),
            foam_upper_cutout,
            foam_lower_cutout,
        });

        scad!(Difference; {
            scad!(Intersection; {
                scad!(Union; {
                    scad!(Difference; {
                        self.interior_cutout(foam_shape_offset),
                    }),
                    front_and_back,
                })
            }),
            self.control_surface_cutout(0.5, 0.95, 30.)
        })

    }

    fn wing_rib_separation(&self, thickness: f32, count: usize) -> f32 {
        // -thickness because we want a rib at the very end of the wing
        (self.wingspan - thickness) / count as f32
    }

    pub fn get_wing_rib(
        &self,
        index: usize,
        thickness: f32,
        count: usize
    ) -> ScadObject {
        let separation = self.wing_rib_separation(thickness, count);

        let offset = separation * index as f32;

        let cutter = scad!(Translate(vec3(0., 0., offset)); {
            centered_cube(vec3(1000., 1000., thickness), (true, true, false))
        });

        scad!(Translate(vec3(0., 0., -offset)); scad!(Intersection; {
            self.extruded_rib_shape(),
            cutter
        }))
    }


    fn interior_cutout(&self, offset: f32) -> ScadObject {
        let extrude_function = |obj| {
            let params = LinExtrudeParams {
                height: self.extrude_thickness,
                .. Default::default()
            };
            scad!(LinearExtrude(params); obj)
        };

        let inner = scad!(Offset(OffsetType::Delta(offset), true); {
            self.exterior_inner_shape()
        });
        let outer = scad!(Offset(OffsetType::Delta(offset), true); {
            self.exterior_outer_shape()
        });

        scad!(Hull; {
            extrude_function(inner),
            scad!(Translate(vec3(0., 0., self.wingspan - self.extrude_thickness)); {
                extrude_function(outer)
            })
        })
    }


    fn exterior_inner_shape(&self) -> ScadObject {
        scad!(Scale2d(vec2(self.inner_length, self.inner_length)); {
            scad!(Polygon(PolygonParameters::new(self.airfoil.points.clone())))
        })
    }

    fn exterior_outer_shape(&self) -> ScadObject {
        scad!(Scale2d(vec2(self.outer_length, self.outer_length)); {
            scad!(Polygon(PolygonParameters::new(self.airfoil.points.clone())))
        })
    }

    fn control_surface_cutout(
        &self,
        start_ratio: f32,
        end_ratio: f32,
        x_size: f32
    ) -> ScadObject {
        let start_z = self.wingspan * start_ratio;

        let full_shape = self.wing_shaped_cutoff_box(
                    CutoffDirection::Both,
                    self.inner_length - x_size,
                    x_size,
                    self.outer_length - x_size,
                    x_size,
                );

        let length = (end_ratio - start_ratio) * self.wingspan;
        let z_cutter = {
            let shape = centered_cube(
                    vec3(1000., 1000., length),
                    (true, true, false)
                );
            let translated = scad!(Translate(vec3(0., 0., start_z));{
                shape
            });

            scad!(Rotate(self.wing_back_angle(), y_axis()); translated)
        };

        scad!(Intersection; {
            full_shape,
            z_cutter,
        })
    }


    fn wing_back_angle(&self) -> f32 {
        - (self.inner_length - self.outer_length).atan2(self.wingspan)
            / std::f32::consts::PI * 180.
    }

    pub fn control_surface(&self, length_ratio: f32, x_size: f32) -> ScadObject {
        let thickness = 4.;
        let points = vec!(
                vec2(0., thickness/2.,),
                vec2(x_size - thickness/2., 0.),
                vec2(thickness, -thickness/2.),
            );

        let extrude_params = LinExtrudeParams {
            height: length_ratio * self.wingspan,
            .. Default::default()
        };

        scad!(LinearExtrude(extrude_params); {
            scad!(Polygon(PolygonParameters::new(points)))
        })
    }
}


// Implementations for various constructs that are not core to design but
// used for debugging
impl Wing {
    pub fn all_wing_ribs(&self, thickness: f32, count: usize) -> ScadObject {
        let separation = self.wing_rib_separation(thickness, count);

        let mut result = scad!(Union);
        for i in 0..count {
            let offset = separation * i as f32;

            result.add_child(scad!(Translate(vec3(0., 0., offset)); {
                self.get_wing_rib(i, thickness, count)
            }));
        }
        result
    }

}

