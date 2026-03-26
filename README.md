# dronePhysics
experimental Sim for some drone configuration ideas
# UAV Simulation – Minimale Spezifikation

## Ziel

Minimale, deterministische Simulation einer starren Drohne zur Analyse beliebiger (nicht-planarer) Antriebskonfigurationen.
Fokus auf strukturellem Verständnis, nicht auf physikalischem Realismus.

---

## Annahmen

* starrer Körper
* konstante Masse und Trägheit
* keine Strukturflexibilität
* keine Aktuatorverzögerungen
* keine komplexe Aerodynamik (initial)

---

## Zustand

Der Systemzustand ist definiert als:

```
x = (p, v, q, ω)
```

* p ∈ ℝ³: Position im World Frame
* v ∈ ℝ³: Geschwindigkeit im World Frame
* q ∈ ℝ⁴: Orientierung als Quaternion (Rotation Body → World)
* ω ∈ ℝ³: Winkelgeschwindigkeit im Body Frame

### Quaternion-Konvention

Quaternions sind in der Reihenfolge `[w, x, y, z]` gespeichert und beschreiben immer die Orientierung **Body → World**.

---

## Koordinatensysteme

* **World Frame**: inertiales Bezugssystem
* **Body Frame**: fest mit der Drohne verbunden

Konvention:

* Thruster-Kräfte sind im Body Frame definiert
* Gravitation wirkt im World Frame
* Integration erfolgt im World Frame

---

## Antriebsmodell

Jeder Antrieb ist definiert durch:

* Position ( r_i \in \mathbb{R}^3 ) (im Body Frame)
* Richtung ( d_i \in \mathbb{R}^3 ), normiert
* Steuersignal ( u_i \in \mathbb{R} )

Kraft:

```
F_i = d_i * u_i
```

Moment:

```
M_i = r_i × F_i
```

---

## Antriebsabbildung (B-Matrix)

Die gesamte Antriebskonfiguration wird durch eine lineare Abbildung beschrieben:

```
w = B * u
```

* w ∈ ℝ⁶: Gesamtwrench (Kraft + Moment)
* u ∈ ℝⁿ: Antriebsvektor
* B ∈ ℝ^(6×n): Konfigurationsmatrix

Struktur von B:

* obere 3 Zeilen: Kraftbeiträge
* untere 3 Zeilen: Momentbeiträge

---

## Dynamik

### Translation

```
m * dv/dt = R(q) * F_body + F_gravity + F_aero
```

### Rotation

```
I * dω/dt = M_body - ω × (Iω)
```

* R(q): Rotationsmatrix aus Quaternion
* I: Trägheitstensor im Body Frame

---

## Kräfte

### Gravitation

```
F_gravity = [0, 0, -m*g]
```

### Aerodynamik (minimal)

```
F_aero ∝ -|v| * v
```

(optional, initial vernachlässigbar)

---

## Struktur des Systems

Trennung in drei Ebenen:

1. **Geometrie (fix)**

   * Positionen und Richtungen der Antriebe

2. **Aktuation (variabel)**

   * Steuersignale u

3. **Dynamik**

   * Bewegung aus Kräften und Momenten

---

## Zentrale Eigenschaften

* gesamte Dynamik basiert auf Summation von Wrenches
* Geometrie ist vollständig in der B-Matrix kodiert
* nicht-planare Anordnungen führen zu:

  * Kopplung von Translation und Rotation
  * potentiell voller 6DOF-Steuerbarkeit

---

## Numerische Simulation

* zeitdiskrete Integration eines kontinuierlichen Systems
* Stabilität abhängig von:

  * Schrittweite
  * korrekter Behandlung der Rotation

---

## Referenztest

Ein einfacher Testfall zur Validierung:

* ein Thruster wirkt exakt entgegen der Gravitation
* keine Momente

Erwartetes Verhalten:

* konstante Position (Schweben)
* keine Rotation

---

## Leitidee

Reduktion des Systems auf:

> lineare Antriebsabbildung (B) + Starrkörperdynamik + einfache Kräfte

Alle Erweiterungen bauen darauf auf.

---

## Implemented API (current repository state)

The repository now includes a small analysis module for actuator allocation:

* `drone_physics.analysis.b_matrix.normalize_direction(direction)`
  * Validates a 3D direction vector and returns a unit vector.
  * Raises `ValueError` for invalid dimensionality or zero-length vectors.
* `drone_physics.analysis.b_matrix.compute_force(direction, command)`
  * Computes per-thruster force using `F_i = d_i * u_i`.
* `drone_physics.analysis.b_matrix.compute_moment(position, force)`
  * Computes per-thruster moment using `M_i = r_i x F_i`.
* `drone_physics.analysis.b_matrix.build_b_matrix(thrusters)`
  * Builds a `6 x n` allocation matrix from an ordered thruster list/configuration.
* `drone_physics.analysis.b_matrix.wrench_from_matrix(B, u)`
  * Computes total wrench using the matrix-vector product `w = B @ u`.
* `drone_physics.analysis.b_matrix.compute_wrench(thrusters, u)`
  * Convenience helper that builds `B` and computes `w`.

### Example

```python
from drone_physics.analysis.b_matrix import build_b_matrix, compute_wrench
from drone_physics.model import Thruster

thrusters = [
    Thruster(position_body=(1.0, 0.0, 0.0), direction_body=(0.0, 0.0, 1.0)),
    Thruster(position_body=(0.0, 1.0, 0.0), direction_body=(0.0, 0.0, 1.0)),
]
u = [2.0, 3.0]

B = build_b_matrix(thrusters)
w = compute_wrench(thrusters, u)
```

Expected wrench in this example:

```text
w = (0.0, 0.0, 5.0, 3.0, -2.0, 0.0)
```
