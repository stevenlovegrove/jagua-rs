use super::QTHazPresence;
use super::assertions;
use super::qt_traits::QTQueryable;
use crate::collision_detection::hazards::filter::NoFilter;
use crate::collision_detection::hazards::{BasicHazardEntity, Hazard, PItemKey};
use crate::collision_detection::{CDEConfig, CDEngine};
use crate::geometry::DTransformation;
use crate::geometry::fail_fast::SPSurrogateConfig;
use crate::geometry::geo_traits::CollidesWith;
use crate::geometry::primitives::{Edge, Point, Rect, SPolygon};
use std::sync::Arc;

#[test]
fn edge_quadrants_allow_f32_false_positive_at_bisector() {
    let rect = Rect::try_new(0.0, 6.1875, 100.0, 106.1875).unwrap();
    let quadrants = rect.quadrants();
    let edge = Edge {
        start: Point(-29.439_144, -32.354_836),
        end: Point(51.902_332, 212.291_02),
    };
    let bisector_y = rect.centroid().1;
    let crossing_fraction =
        -f64::from(edge.start.0) / (f64::from(edge.end.0) - f64::from(edge.start.0));
    let crossing_y = f64::from(edge.start.1)
        + crossing_fraction * (f64::from(edge.end.1) - f64::from(edge.start.1));
    let bisector_ulp = f32::from_bits(bisector_y.to_bits() + 1) - bisector_y;

    assert!(crossing_y > f64::from(bisector_y));
    assert!(crossing_y - f64::from(bisector_y) < f64::from(bisector_ulp));

    assert_eq!(
        quadrants.map(|quadrant| edge.collides_with(&quadrant)),
        [false, true, true, false]
    );
    assert_eq!(
        assertions::edge_collisions_with_rects_f64(&edge, quadrants.each_ref()),
        [false, true, false, false]
    );
    assert_eq!(
        edge.collides_with_quadrants(&rect, quadrants.each_ref()),
        [false, true, false, false]
    );
}

fn engine_with_item(item: Rect) -> CDEngine {
    let root = Rect::try_new(0.0, 0.0, 100.0, 100.0).unwrap();
    let config = CDEConfig {
        quadtree_depth: 3,
        cd_threshold: 0,
        item_surrogate_config: SPSurrogateConfig {
            n_pole_limits: [(0, 0.0); 3],
            ff_pole_area_ratio: 0.0,
            n_ff_piers: 0,
        },
    };
    let exterior = Hazard::new(BasicHazardEntity::Exterior, Arc::new(SPolygon::from(root)));
    let mut cde = CDEngine::new(root, vec![exterior], config);
    let entity = BasicHazardEntity::PlacedItem {
        id: 0,
        dt: DTransformation::empty(),
        pk: PItemKey::default(),
    };
    cde.register_hazard(Hazard::new(entity, Arc::new(SPolygon::from(item))));
    cde
}

fn item_presences_in_root_children(cde: &CDEngine) -> Vec<&QTHazPresence> {
    let children = cde.quadtree.children.as_ref().unwrap();
    children
        .iter()
        .flat_map(|child| child.hazards.iter())
        .filter(|hz| hz.entity != BasicHazardEntity::Exterior)
        .map(|hz| &hz.presence)
        .collect()
}

#[test]
fn hazard_outside_the_root_is_absent_from_its_children() {
    let cde = engine_with_item(Rect::try_new(150.0, 0.0, 160.0, 10.0).unwrap());

    assert!(item_presences_in_root_children(&cde).is_empty());
    let inside = SPolygon::from(Rect::try_new(40.0, 40.0, 60.0, 60.0).unwrap());
    assert!(!cde.detect_poly_collision(&inside, &NoFilter));
}

#[test]
fn hazard_around_the_root_is_entirely_present_in_its_children() {
    let cde = engine_with_item(Rect::try_new(-10.0, -10.0, 110.0, 110.0).unwrap());

    let presences = item_presences_in_root_children(&cde);
    assert_eq!(presences.len(), 4);
    assert!(presences.iter().all(|p| matches!(p, QTHazPresence::Entire)));
    let inside = SPolygon::from(Rect::try_new(40.0, 40.0, 60.0, 60.0).unwrap());
    assert!(cde.detect_poly_collision(&inside, &NoFilter));
}
