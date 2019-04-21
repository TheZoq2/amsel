use scad::*;
use scad_util::constants::x_axis;

fn mid_section(mid_y_size: f32, outer_y_size: f32) -> ScadObject {
    let points = vec!(
            vec2(0., mid_y_size),
            vec2(outer_y_size, mid_y_size),
            vec2(outer_y_size, -mid_y_size),
            vec2(0., -mid_y_size),
        ).iter().map(|x| x/2.).collect();

    scad!(Polygon(PolygonParameters::new(points)))
}


fn teeth(amount: usize, outer_y_size: f32, separation: f32) -> ScadObject {
    let inner_y_size = outer_y_size - 1.;
    let mut points = vec!();
    let mut x = 0.;
    for _ in 0..amount {
        points.push(vec2(x, outer_y_size));
        x += separation;
        points.push(vec2(x, inner_y_size));
    }
    for _ in 0..amount {
        points.push(vec2(x, -inner_y_size));
        x -= separation;
        points.push(vec2(x, -outer_y_size));
    }
    points = points.iter().map(|x| x / 2.).collect();
    scad!(Polygon(PolygonParameters::new(points)))
}

pub fn tooth_cutout(amount: usize, outer_height: f32, thickness: f32) -> ScadObject {
    let params = LinExtrudeParams {
        height: thickness,
        .. Default::default()
    };
    scad!(LinearExtrude(params); {
        teeth(amount, outer_height + 0.5, outer_height)
    })
}


pub fn hinge(tooth_amount: usize, outer_height: f32, thickness: f32) -> ScadObject{
    let params = LinExtrudeParams {
        height: thickness,
        .. Default::default()
    };
    let shape = scad!(LinearExtrude(params); {
        mid_section(1., outer_height),
        scad!(Translate2d(vec2(outer_height / 2., 0.)); {
            teeth(tooth_amount, outer_height, outer_height)
        })
    });

    scad!(Union; {
        shape.clone(),
        scad!(Mirror(x_axis()); shape)
    })
}


pub fn test_mount() -> ScadObject {
    scad!(Difference; {
        centered_cube(vec3(12., 8., 6.), (false, true, false)),
        scad!(Translate(vec3(0., 0., 2.)); {
            tooth_cutout(5, 3., 4.)
        })
    })
}

pub fn test_hinge() -> ScadObject {
    hinge(5, 3., 4.)
}
