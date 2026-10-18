// Copyright 2026 Héctor Azpúrua
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Obstacles: OccupancyGrid, EuclideanDistanceTransform, ObstacleMap, the footprint and its
// checker, and the CollisionPredictor (occupancy.hpp, footprint.hpp, collision_predictor.hpp).
//
// Grids are (height, width) NumPy arrays indexed [iy, ix]; cell (ix, iy) covers
// [origin + (ix, iy) res, origin + (ix + 1, iy + 1) res) in the grid frame (Costmap2D layout).

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "bindings.hpp"

namespace vectorfield_py
{

namespace
{

using vf::EuclideanDistanceTransform;
using vf::OccupancyGrid;
using GridPtr = std::shared_ptr<OccupancyGrid>;
using EdtPtr = std::shared_ptr<EuclideanDistanceTransform>;

void setMask(
  OccupancyGrid & grid, const MaskArray & mask, double resolution, const vf::Vec2 & origin)
{
  if (mask.ndim() != 2) {
    throw py::value_error("mask must be a 2-D (height, width) array");
  }
  const auto height = static_cast<int>(mask.shape(0));
  const auto width = static_cast<int>(mask.shape(1));
  const vf::Span<const std::uint8_t> cells(mask.data(), static_cast<std::size_t>(mask.size()));
  if (!grid.setFromMask(cells, width, height, resolution, origin)) {
    throw py::value_error(
      "invalid grid: needs a non-empty mask, a positive finite resolution and a finite origin");
  }
}

void requireCell(const EuclideanDistanceTransform & edt, int ix, int iy)
{
  if (ix < 0 || iy < 0 || ix >= edt.width() || iy >= edt.height()) {
    throw py::index_error(
      "cell (" + std::to_string(ix) + ", " + std::to_string(iy) + ") is outside the transform");
  }
}

void requireUpToDate(const OccupancyGrid & grid, const EuclideanDistanceTransform & edt)
{
  if (edt.builtVersion() != grid.version()) {
    throw py::value_error("edt is not EDT_all of grid: call edt.update(grid) first");
  }
}

void bindGrid(py::module_ & m)
{
  py::class_<OccupancyGrid, GridPtr>(
    m, "OccupancyGrid",
    "Obstacle sites of a costmap: a (height, width) mask (nonzero = site), a resolution (m) and "
    "the grid-frame origin of the corner of cell (0, 0).")
    .def(py::init([]() { return std::make_shared<OccupancyGrid>(); }), "An empty grid.")
    .def(
      py::init([](const MaskArray & mask, double resolution, const vf::Vec2 & origin) {
        auto grid = std::make_shared<OccupancyGrid>();
        setMask(*grid, mask, resolution, origin);
        return grid;
      }),
      py::arg("mask"), py::arg("resolution"), py::arg("origin") = vf::Vec2(vf::Vec2::Zero()),
      "A grid from a (height, width) mask; raises ValueError on inconsistent arguments.")
    .def(
      "set_from_mask", &setMask, py::arg("mask"), py::arg("resolution"),
      py::arg("origin") = vf::Vec2(vf::Vec2::Zero()),
      "Replaces the content; version changes iff the geometry or a site flag changed.")
    .def("reserve", &OccupancyGrid::reserve, py::arg("width"), py::arg("height"))
    .def_property_readonly("width", &OccupancyGrid::width, "Cells along x.")
    .def_property_readonly("height", &OccupancyGrid::height, "Cells along y.")
    .def_property_readonly(
      "shape", [](const OccupancyGrid & g) { return py::make_tuple(g.height(), g.width()); },
      "(height, width).")
    .def_property_readonly("resolution", &OccupancyGrid::resolution, "Cell size (m).")
    .def_property_readonly(
      "origin", [](const OccupancyGrid & g) -> vf::Vec2 { return g.origin(); },
      "Grid-frame corner of cell (0, 0).")
    .def_property_readonly(
      "mask", [](const OccupancyGrid & g) { return fromMask(g.mask(), g.width(), g.height()); },
      "The mask (height, width) uint8, 1 = site.")
    .def_property_readonly("version", &OccupancyGrid::version, "Content stamp (0 when empty).")
    .def("in_bounds", &OccupancyGrid::inBounds, py::arg("ix"), py::arg("iy"))
    .def("is_site", &OccupancyGrid::isSite, py::arg("ix"), py::arg("iy"))
    .def("cell_center", &OccupancyGrid::cellCenter, py::arg("ix"), py::arg("iy"))
    .def(
      "cell_at",
      [](const OccupancyGrid & g, const vf::Vec2 & x) -> std::optional<std::pair<int, int>> {
        int ix = 0;
        int iy = 0;
        if (!g.cellAt(x, &ix, &iy)) {
          return std::nullopt;
        }
        return std::make_pair(ix, iy);
      },
      py::arg("x"), "(ix, iy) of the cell containing a grid-frame point, or None outside.");
}

py::array_t<double> edtGrid(
  const EuclideanDistanceTransform & edt,
  double (EuclideanDistanceTransform::*value)(int, int) const noexcept)
{
  const std::vector<py::ssize_t> shape{edt.height(), edt.width()};
  py::array_t<double> out(shape);
  auto view = out.mutable_unchecked<2>();
  for (int iy = 0; iy < edt.height(); ++iy) {
    for (int ix = 0; ix < edt.width(); ++ix) {
      view(iy, ix) = (edt.*value)(ix, iy);
    }
  }
  return out;
}

void bindEdt(py::module_ & m)
{
  StructBinder<vf::ObstacleQuery>(m, "ObstacleQuery", "Result of a continuous obstacle query.")
    .field("D_o", &vf::ObstacleQuery::D_o, "Distance (m); inf when invalid.")
    .field("n_o", &vf::ObstacleQuery::n_o, "Normal away from the obstacle.")
    .field("x_o", &vf::ObstacleQuery::x_o, "Nearest obstacle point (grid frame).")
    .field("valid", &vf::ObstacleQuery::valid, "False without sites or outside the grid.")
    .finish();

  py::class_<EuclideanDistanceTransform, EdtPtr>(
    m, "EuclideanDistanceTransform",
    "Exact Euclidean distance transform with feature transform (Felzenszwalb-Huttenlocher) "
    "between cell centres, over all sites of a grid or a filtered subset.")
    .def(
      py::init([]() { return std::make_shared<EuclideanDistanceTransform>(); }),
      "An empty transform.")
    .def(
      py::init([](const OccupancyGrid & grid) {
        auto edt = std::make_shared<EuclideanDistanceTransform>();
        edt->update(grid);
        return edt;
      }),
      py::arg("grid"), "The transform over all sites of `grid`.")
    .def("reserve", &EuclideanDistanceTransform::reserve, py::arg("width"), py::arg("height"))
    .def(
      "update", &EuclideanDistanceTransform::update, py::arg("grid"),
      py::call_guard<py::gil_scoped_release>(),
      "Rebuilds over all sites iff the grid's version changed; True iff it rebuilt.")
    .def(
      "build",
      [](
        EuclideanDistanceTransform & edt, const OccupancyGrid & grid,
        const std::optional<MaskArray> & filter) {
        if (!filter) {
          const py::gil_scoped_release release;
          edt.build(grid, {});
          return;
        }
        if (
          filter->ndim() != 2 || filter->shape(0) != grid.height() ||
          filter->shape(1) != grid.width()) {
          throw py::value_error("site_filter must have the grid's (height, width) shape");
        }
        const MaskArray & f = *filter;
        const py::gil_scoped_release release;
        edt.build(grid, vf::Span<const std::uint8_t>(f.data(), static_cast<std::size_t>(f.size())));
      },
      py::arg("grid"), py::arg("site_filter") = py::none(),
      "Rebuilds unconditionally over the sites with a nonzero (height, width) filter entry.")
    .def_property_readonly("has_sites", &EuclideanDistanceTransform::hasSites)
    .def_property_readonly("width", &EuclideanDistanceTransform::width)
    .def_property_readonly("height", &EuclideanDistanceTransform::height)
    .def_property_readonly("resolution", &EuclideanDistanceTransform::resolution)
    .def_property_readonly(
      "origin", [](const EuclideanDistanceTransform & e) -> vf::Vec2 { return e.origin(); })
    .def_property_readonly(
      "built_version", &EuclideanDistanceTransform::builtVersion,
      "Grid version of the last build (0 if filtered or never built).")
    .def(
      "squared_distance_cells",
      [](const EuclideanDistanceTransform & e, int ix, int iy) {
        requireCell(e, ix, iy);
        return e.squaredDistanceCells(ix, iy);
      },
      py::arg("ix"), py::arg("iy"), "Squared distance to the nearest site, in cells^2.")
    .def(
      "distance",
      [](const EuclideanDistanceTransform & e, int ix, int iy) {
        requireCell(e, ix, iy);
        return e.distance(ix, iy);
      },
      py::arg("ix"), py::arg("iy"), "Distance from a cell centre to the nearest site (m).")
    .def(
      "feature",
      [](const EuclideanDistanceTransform & e, int ix, int iy) {
        requireCell(e, ix, iy);
        if (!e.hasSites()) {
          throw py::value_error("the transform has no sites");
        }
        int fx = 0;
        int fy = 0;
        e.feature(ix, iy, &fx, &fy);
        return py::make_tuple(fx, fy);
      },
      py::arg("ix"), py::arg("iy"), "(fx, fy) of the nearest site cell.")
    .def(
      "distance_grid",
      [](const EuclideanDistanceTransform & e) {
        return edtGrid(e, &EuclideanDistanceTransform::distance);
      },
      "Distances (height, width) in metres (inf without sites).")
    .def(
      "squared_distance_grid",
      [](const EuclideanDistanceTransform & e) {
        return edtGrid(e, &EuclideanDistanceTransform::squaredDistanceCells);
      },
      "Squared distances (height, width) in cells^2.")
    .def(
      "feature_grid",
      [](const EuclideanDistanceTransform & e) {
        if (!e.hasSites()) {
          throw py::value_error("the transform has no sites");
        }
        const std::vector<py::ssize_t> shape{e.height(), e.width(), 2};
        py::array_t<std::int32_t> out(shape);
        auto view = out.mutable_unchecked<3>();
        for (int iy = 0; iy < e.height(); ++iy) {
          for (int ix = 0; ix < e.width(); ++ix) {
            int fx = 0;
            int fy = 0;
            e.feature(ix, iy, &fx, &fy);
            view(iy, ix, 0) = fx;
            view(iy, ix, 1) = fy;
          }
        }
        return out;
      },
      "Feature cells (height, width, 2) as [fx, fy].")
    .def("query", &EuclideanDistanceTransform::query, py::arg("x_grid"), "Continuous query.")
    .def(
      "query_points",
      [](const EuclideanDistanceTransform & e, const DoubleArray & points) {
        const std::vector<vf::Vec2> xs = toPoints(points, "points");
        std::vector<vf::ObstacleQuery> qs(xs.size());
        {
          const py::gil_scoped_release release;
          for (std::size_t i = 0; i < xs.size(); ++i) {
            qs[i] = e.query(xs[i]);
          }
        }
        const auto n = static_cast<py::ssize_t>(qs.size());
        py::array_t<double> D(n);
        py::array_t<bool> valid(n);
        std::vector<vf::Vec2> normals(qs.size());
        std::vector<vf::Vec2> nearest(qs.size());
        for (std::size_t i = 0; i < qs.size(); ++i) {
          D.mutable_at(static_cast<py::ssize_t>(i)) = qs[i].D_o;
          valid.mutable_at(static_cast<py::ssize_t>(i)) = qs[i].valid;
          normals[i] = qs[i].n_o;
          nearest[i] = qs[i].x_o;
        }
        py::dict out;
        out["D_o"] = D;
        out["n_o"] = fromPoints(normals);
        out["x_o"] = fromPoints(nearest);
        out["valid"] = valid;
        return out;
      },
      py::arg("points"),
      "query() of each row of an (N, 2) array: a dict of D_o, valid (N,) and n_o, x_o (N, 2).");

  m.def(
    "flood_fill_dilated_component",
    [](const EuclideanDistanceTransform & edt, double radius, int seed_x, int seed_y) {
      std::vector<std::uint8_t> component;
      std::vector<int> queue;
      vf::floodFillDilatedComponent(edt, radius, seed_x, seed_y, &component, &queue);
      component.resize(
        static_cast<std::size_t>(edt.width()) * static_cast<std::size_t>(edt.height()));
      return fromMask(component, edt.width(), edt.height());
    },
    py::arg("edt_all"), py::arg("dilation_radius"), py::arg("seed_x"), py::arg("seed_y"),
    "K: the 8-connected component of {EDT <= radius} containing the seed, as a (height, width) "
    "mask.");
  m.def(
    "nearest_dilated_cell",
    [](const EuclideanDistanceTransform & edt, double radius, const vf::Vec2 & x, double search)
      -> std::optional<std::pair<int, int>> {
      int cx = 0;
      int cy = 0;
      if (!vf::nearestDilatedCell(edt, radius, x, search, &cx, &cy)) {
        return std::nullopt;
      }
      return std::make_pair(cx, cy);
    },
    py::arg("edt_all"), py::arg("dilation_radius"), py::arg("x_grid"), py::arg("search_radius"),
    "(cx, cy) of the nearest dilated cell within search_radius, or None.");

  py::class_<ObstacleMapBox>(
    m, "ObstacleMap",
    "The obstacle input of a Tracker step: a grid, its distance transform EDT_all (built from "
    "the grid when not given) and T_grid_eval (evaluation frame to grid frame). The transform is "
    "brought up to date with the grid before every use.")
    .def(
      py::init(
        [](const GridPtr & grid, const std::optional<EdtPtr> & edt, const vf::Isometry2 & T) {
          if (!grid) {
            throw py::value_error("grid must not be None");
          }
          ObstacleMapBox box{
            grid, edt && *edt ? *edt : std::make_shared<EuclideanDistanceTransform>(), T};
          box.edt->update(*grid);
          return box;
        }),
      py::arg("grid"), py::arg("edt") = py::none(), py::arg("T_grid_eval") = vf::Isometry2{})
    .def_readonly("grid", &ObstacleMapBox::grid, "The OccupancyGrid (shared).")
    .def_readonly("edt", &ObstacleMapBox::edt, "The EuclideanDistanceTransform (shared).")
    .def_readwrite("T_grid_eval", &ObstacleMapBox::T_grid_eval, "Evaluation frame to grid frame.")
    .def(
      "update", [](const ObstacleMapBox & b) { return b.edt->update(*b.grid); },
      "Brings the transform up to date with the grid; True iff it rebuilt.");
}

void bindFootprint(py::module_ & m)
{
  using vf::Footprint;
  py::class_<Footprint>(
    m, "Footprint",
    "Robot footprint (robot frame): a filled polygon or a disc, with w_lat, r_insc and r_circ.")
    .def(py::init<>(), "A point robot.")
    .def_static("circle", &Footprint::circle, py::arg("radius"), "A disc; raises ConfigError.")
    .def_static(
      "polygon",
      [](const DoubleArray & vertices) {
        const std::vector<vf::Vec2> pts = toPoints(vertices, "vertices");
        return Footprint::polygon(pts);
      },
      py::arg("vertices"), "A filled polygon (N, 2); raises ConfigError.")
    .def_static("from_spec", &Footprint::fromSpec, py::arg("spec"), "From a FootprintSpec.")
    .def_property_readonly("is_circle", &Footprint::isCircle)
    .def_property_readonly("radius", &Footprint::radius)
    .def_property_readonly(
      "vertices", [](const Footprint & f) { return fromPoints(f.vertices()); }, "Polygon (N, 2).")
    .def_property_readonly("radii", &Footprint::radii, py::return_value_policy::copy)
    .def_property_readonly("w_lat", &Footprint::wLat)
    .def_property_readonly("r_insc", &Footprint::rInsc)
    .def_property_readonly("r_circ", &Footprint::rCirc);

  py::enum_<vf::FootprintDecision>(m, "FootprintDecision", "Which rule decided a check.")
    .value("FREE_SHORTCUT", vf::FootprintDecision::kFreeShortcut)
    .value("COLLISION_SHORTCUT", vf::FootprintDecision::kCollisionShortcut)
    .value("FREE_RASTERIZED", vf::FootprintDecision::kFreeRasterized)
    .value("COLLISION_RASTERIZED", vf::FootprintDecision::kCollisionRasterized)
    .value("OFF_GRID", vf::FootprintDecision::kOffGrid);

  using vf::FootprintChecker;
  py::class_<FootprintChecker>(
    m, "FootprintChecker",
    "Footprint collision test against a grid (poses in the grid frame); `edt` must be EDT_all "
    "of the grid (edt.update(grid)).")
    .def(py::init<Footprint>(), py::arg("footprint") = Footprint{})
    .def_property_readonly("footprint", &FootprintChecker::footprint, py::return_value_policy::copy)
    .def(
      "collides",
      [](
        const FootprintChecker & c, const vf::Pose2 & pose, const OccupancyGrid & grid,
        const EuclideanDistanceTransform & edt) {
        requireUpToDate(grid, edt);
        return c.collides(pose, grid, edt);
      },
      py::arg("pose_grid"), py::arg("grid"), py::arg("edt"), "True iff the pose collides.")
    .def(
      "classify",
      [](
        const FootprintChecker & c, const vf::Pose2 & pose, const OccupancyGrid & grid,
        const EuclideanDistanceTransform & edt) {
        requireUpToDate(grid, edt);
        return c.classify(pose, grid, edt);
      },
      py::arg("pose_grid"), py::arg("grid"), py::arg("edt"), "The FootprintDecision.")
    .def(
      "collides_ground_truth", &FootprintChecker::collidesGroundTruth, py::arg("pose_grid"),
      py::arg("grid"), "Rasterised ground truth.")
    .def(
      "off_grid", &FootprintChecker::offGrid, py::arg("pose_grid"), py::arg("grid"),
      "True iff the disc of radius r_circ + res around the pose leaves the grid.");
}

void bindPredictor(py::module_ & m)
{
  using vf::RolloutStep;
  StructBinder<RolloutStep>(m, "RolloutStep", "One nominal step proposed by a rollout policy.")
    .field("cmd", &RolloutStep::cmd, "Command held for the next period.")
    .field("phase", &RolloutStep::phase, "Phase of the step.")
    .field("settled", &RolloutStep::settled, "The rollout stops after this step.")
    .finish();

  using vf::RolloutOptions;
  StructBinder<RolloutOptions>(m, "RolloutOptions", "Options of one rollout.")
    .field("t0", &RolloutOptions::t0, "Start time (s).")
    .field("period", &RolloutOptions::period, "Control period (s).")
    .field("horizon", &RolloutOptions::horizon, "Horizon T_h (s).")
    .field("through_cancel", &RolloutOptions::through_cancel, "Continue through a cancel ramp.")
    .finish();

  using vf::RolloutResult;
  StructBinder<RolloutResult>(m, "RolloutResult", "Outcome of one rollout (math_spec 10.2).")
    .field("s_free", &RolloutResult::s_free, "Free arc length (m); inf if collision-free.")
    .field("first_collision", &RolloutResult::first_collision, "Index of the first collision.")
    .field("collided", &RolloutResult::collided, "A checked pose collided.")
    .field(
      "collided_in_rotation", &RolloutResult::collided_in_rotation,
      "The first collision happened in a rotation phase.")
    .field("off_grid", &RolloutResult::off_grid, "The footprint left the grid.")
    .field("reached_horizon", &RolloutResult::reached_horizon, "The horizon was reached.")
    .field("settled", &RolloutResult::settled, "A settled step ended the rollout.")
    .field("stopped", &RolloutResult::stopped, "The rollout reached STOPPED.")
    .field("steps", &RolloutResult::steps, "Control periods simulated.")
    .finish();

  using vf::CollisionPredictor;
  py::class_<CollisionPredictor>(
    m, "CollisionPredictor",
    "Zero-order-hold rollout with exact sub-stepped integration on the nominal plant.")
    .def(
      py::init<const vf::Kinematics &, const vf::FootprintChecker &>(), py::arg("plant"),
      py::arg("checker"))
    .def(
      "rollout",
      [](
        const CollisionPredictor & p, const vf::Pose2 & start, const ObstacleMapBox & map,
        const RolloutOptions & options, const py::function & policy) {
        CallbackGuard guard;
        RolloutStep stop;
        stop.phase = vf::Phase::kStopped;
        stop.settled = true;
        const auto call = [&policy, &guard, &stop](const vf::Pose2 & pose, double t) {
          return guard.call([&]() { return policy(pose, t).cast<RolloutStep>(); }, stop);
        };
        std::vector<vf::Pose2> poses;
        const RolloutResult result = p.rollout(start, map.map(), options, call, &poses);
        guard.rethrow();
        return py::make_tuple(result, fromPoses(poses));
      },
      py::arg("start"), py::arg("obstacles"), py::arg("options"), py::arg("policy"),
      "Runs policy(pose, t) -> RolloutStep from `start`: (RolloutResult, checked poses (N, 3)).")
    .def_property_readonly(
      "plant", &CollisionPredictor::plant, py::return_value_policy::copy, "Kinematics.")
    .def_property_readonly(
      "checker", &CollisionPredictor::checker, py::return_value_policy::copy, "FootprintChecker.");

  m.def(
    "braking_cap", &vf::brakingCap, py::arg("s_free"), py::arg("stop_margin"), py::arg("a_dec"),
    "v_cap = sqrt(2 a_dec max(0, s_free - stop_margin)); inf for inf s_free.");
}

}  // namespace

void bindObstacles(py::module_ & m)
{
  bindGrid(m);
  bindEdt(m);
  bindFootprint(m);
  bindPredictor(m);
}

}  // namespace vectorfield_py
