use crate::error::SimError;
use egui_macroquad::macroquad::prelude::*;

const DT: f32 = 0.15;
const G: f32 = 18.0;
const NODE_RADIUS: f32 = 6.0;
const ROPE_WIDTH: f32 = 4.0;
const TARGET_DIST: f32 = 50.0;
const RIGIDITY: f32 = 1.0;
const DRAG: f32 = 0.5;

const NUM_POINTS: usize = 10;

#[derive(Copy, Clone, Debug)]
pub struct Node {
    last_pos: Vec2,
    pos: Vec2,
    vel: Vec2,
    force: Vec2,
    mass: f32,
    fixed: bool,
}

impl Default for Node {
    fn default() -> Self {
        Self {
            pos: Default::default(),
            last_pos: Default::default(),
            vel: Default::default(),
            force: Default::default(),
            mass: 1.0,
            fixed: Default::default(),
        }
    }
}

impl Node {
    pub fn with_pos_and_mass(pos: Vec2, mass: f32) -> Node {
        Node {
            pos,
            last_pos: pos,
            mass,
            ..Node::default()
        }
    }

    pub fn integrate(&mut self) {
        if self.fixed {
            return;
        }

        let acc = self.force / self.mass;

        self.last_pos = self.pos;
        self.vel = self.vel + acc * DT;
        self.pos = self.pos + self.vel * DT;
    }

    pub fn differentiate(&mut self) {
        if self.fixed {
            return;
        }

        self.vel = (self.pos - self.last_pos) / DT;
        self.force = Vec2::ZERO;
    }

    pub fn apply_gravity(&mut self) {
        if self.fixed {
            return;
        }

        self.force += Vec2::new(0.0, G * self.mass);
    }

    pub fn apply_drag(&mut self) {
        if self.fixed {
            return;
        }

        self.force += -self.vel * DRAG;
    }

    pub fn add_offs(&mut self, offs: Vec2) {
        if !self.fixed {
            self.pos += offs;
        }
    }
}

pub struct Constraint {
    a: usize,
    b: usize,
    break_threshold: f32,
}

impl Constraint {
    pub fn solve(&self, arena: &mut Vec<Node>) {
        let (a_offs, b_offs) = {
            let a = &arena[self.a];
            let b = &arena[self.b];

            let r = b.pos - a.pos;
            let dist = r.length();

            let norm = r.normalize_or_zero();
            let diff = dist - TARGET_DIST;
            let mut offs = norm * diff * RIGIDITY / (a.mass + b.mass);

            if dist < TARGET_DIST {
                offs *= 0.5;
            }

            (offs / a.mass, -offs / b.mass)
        };

        arena[self.a].add_offs(a_offs);
        arena[self.b].add_offs(b_offs);
    }
}

pub struct MainState {
    arena: Vec<Node>,
    constraints: Vec<Constraint>,
    last_mouse_pos: Vec2,
}

impl MainState {
    pub fn apply_wind(&mut self) {
        // disable wind when knife is on
        if is_mouse_button_down(MouseButton::Right) {
            return
        }

        let current_mouse_pos: Vec2 = mouse_position().into();
        for node in self.arena.iter_mut() {
            if (node.pos - current_mouse_pos).length() < 30.0 {
                let f = current_mouse_pos - self.last_mouse_pos;
                node.force += f * 50.0;
            }
        }
    }

    pub fn solve_constraints(&mut self) {
        for _ in 0..5 {
            for constraint in self.constraints.iter() {
                constraint.solve(&mut self.arena);
            }
        }
    }

    pub fn update(&mut self) -> Result<(), SimError> {
        self.arena.iter_mut().for_each(Node::apply_gravity);
        self.arena.iter_mut().for_each(Node::apply_drag);
        self.apply_wind();
        self.arena.iter_mut().for_each(Node::integrate);
        self.solve_constraints();
        self.constraints.retain(|constraint| {
            (self.arena[constraint.a].pos - self.arena[constraint.b].pos).length() < constraint.break_threshold
        });
        if is_mouse_button_down(MouseButton::Right) {
            let mouse_pos: Vec2 = mouse_position().into();
            self.constraints.retain(|constraint| {
                // https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect
                let a = self.arena[constraint.a].pos;
                let b = self.arena[constraint.b].pos;
                let c = mouse_pos;
                let d = self.last_mouse_pos;

                fn ccw(a: Vec2, b: Vec2, c: Vec2) -> bool {
                    (c.y-a.y) * (b.x-a.x) > (b.y-a.y) * (c.x-a.x)
                }

                let intersects = (ccw(a, c, d) != ccw(b, c, d)) && (ccw(a, b, c) != ccw(a, b, d));
                !intersects
            });
        }
        self.arena.iter_mut().for_each(Node::differentiate);
        self.last_mouse_pos = mouse_position().into();

        Ok(())
    }

    pub fn draw(&mut self) -> Result<(), SimError> {
        for constraint in self.constraints.iter() {
            let a = self.arena[constraint.a];
            let b = self.arena[constraint.b];
            draw_line(a.pos.x, a.pos.y, b.pos.x, b.pos.y, ROPE_WIDTH, WHITE);
        }

        for node in self.arena.iter() {
            let c = if node.fixed { RED } else { WHITE };
            draw_circle(node.pos.x, node.pos.y, NODE_RADIUS, c);
        }

        draw_text("Right Click to Cut", 10.0, screen_height() - 50.0, 36.0, WHITE);

        Ok(())
    }
}

impl Default for MainState {
    fn default() -> Self {
        let mut arena = Vec::new();
        let mut constraints = Vec::new();

        let y_offs = screen_height() / 5.0;

        let one_third = screen_width() / 3.0;
        let two_thirds = screen_width() * 2.0 / 3.0;

        for i in 0..NUM_POINTS {
            arena.push(Node::with_pos_and_mass(
                Vec2::new(one_third, y_offs + TARGET_DIST * i as f32),
                1.0 + (i as f32 / 20.0).powi(2) * 0.0,
            ));

            if i == 0 {
                arena[i].fixed = true;
            }

            if i > 0 {
                constraints.push(Constraint {
                    a: i - 1,
                    b: i,
                    break_threshold: TARGET_DIST * 5.0,
                });
            }
        }

        Self {
            arena,
            constraints,
            last_mouse_pos: mouse_position().into(),
        }
    }
}
