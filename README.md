# ProcShapes

ProcShapes is a compact procedural geometry project that generates simple 3D furniture-like shapes from parameter vectors. It defines reusable parameter spaces and turns them into mesh objects for categories such as beds, chairs, storage units, and tables.

## Project overview

The repository contains:

- `paramvectordef.py` - parameter definitions and random vector generation
- `procedure.py` - procedural mesh generation logic for different furniture families
- `requirements.txt` - Python dependencies for mesh processing and visualization

## Supported categories

The generator currently supports:

- `bed`
- `chair`
- `storage`
- `table`

Each category uses a structured parameter vector that encodes dimensions, style choices, and structural options such as leg type, back type, and storage layout.

## Dependencies

The project uses:

- `bpy==5.0.0`
- `mathutils==5.1.0`
- `numpy==2.5.3`
- `open3d==0.19.0`
- `tqdm==4.67.1`
- `trimesh==4.12.2`

## Installation

Install the Python dependencies:

```bash
pip install -r requirements.txt
```

Note: this project imports `bpy` and `bmesh`, which are part of Blender's Python environment. Running `procedure.py` outside Blender may require a Blender-compatible Python setup.

## How it works

1. A category is selected via `get_param_vec_def(category)`.
2. A parameter vector definition defines scalar and categorical parameters.
3. Random or explicit vectors are sampled and encoded/decoded.
4. `Procedure.get_shape(paramvector)` converts that parameter vector into a mesh.
5. The generated mesh is normalized and returned as a `trimesh.Trimesh` object.

## Usage

Run the example generator:

```bash
python procedure.py --num_samples 5
```

This will create random example meshes for the default test category (`storage`) and display them using Open3D.

You can also use the parameter definitions directly in Python:

```python
from paramvectordef import get_param_vec_def

param_def = get_param_vec_def('chair')
vectors = param_def.get_random_vectors(10)
print(vectors)
```
