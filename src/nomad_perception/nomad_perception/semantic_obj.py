"""Partition an OBJ's faces into semantic classes without changing its geometry.

Only face records and material-library references are filtered/replaced. All vertex,
normal and UV records remain at their original positions. In particular, moving all
vertices to a new header would silently change interleaved negative face indices;
this implementation deliberately does not do that.

The source OBJ, MTLs and textures are read-only. Generated MTLs retain material
attributes and reference the original textures through absolute filesystem paths.
The resulting assets are therefore local overlays, not portable exports.
"""

from __future__ import annotations

from collections import Counter
from pathlib import Path
import re
import shlex
from urllib.parse import unquote, urlparse


class ObjSemanticsError(ValueError):
    """An OBJ cannot be partitioned safely with the supplied material mapping."""


_TEXTURE_DIRECTIVES = {"bump", "disp", "decal", "refl", "norm"}
_ONE_ARGUMENT_OPTIONS = {
    "-blendu", "-blendv", "-boost", "-texres", "-clamp", "-bm",
    "-imfchan", "-type", "-cc", "-colorspace",
}
_VARIABLE_NUMBER_OPTIONS = {"-o", "-s", "-t"}


def _parts(line: str) -> tuple[str, str]:
    stripped = line.strip()
    if not stripped or stripped.startswith("#"):
        return "", ""
    parts = stripped.split(None, 1)
    return parts[0], parts[1] if len(parts) == 2 else ""


def _tokens(value: str, *, location: str) -> list[str]:
    try:
        return shlex.split(value, comments=True, posix=True)
    except ValueError as exc:
        raise ObjSemanticsError(f"{location}: invalid quoting: {exc}") from exc


def _local_path(value: str, parent: Path) -> Path:
    if value.startswith("file:"):
        parsed = urlparse(value)
        if parsed.netloc not in ("", "localhost"):
            raise ObjSemanticsError(f"Non-local file URI is unsupported: {value}")
        return Path(unquote(parsed.path)).resolve()
    if "://" in value:
        raise ObjSemanticsError(f"Non-local asset reference is unsupported: {value}")
    path = Path(value)
    return (path if path.is_absolute() else parent / path).resolve()


def _is_number(value: str) -> bool:
    try:
        float(value)
        return True
    except ValueError:
        return False


def _rewrite_texture(line: str, mtl_path: Path, line_number: int) -> str:
    directive, payload = _parts(line)
    tokens = _tokens(payload, location=f"{mtl_path}:{line_number}")
    cursor = 0
    while cursor < len(tokens) and tokens[cursor].startswith("-"):
        option = tokens[cursor]
        cursor += 1
        if option in _VARIABLE_NUMBER_OPTIONS:
            start = cursor
            while cursor < len(tokens) and cursor - start < 3 and _is_number(tokens[cursor]):
                cursor += 1
            if cursor == start:
                raise ObjSemanticsError(f"{mtl_path}:{line_number}: {option} needs a number")
        else:
            count = 2 if option == "-mm" else 1 if option in _ONE_ARGUMENT_OPTIONS else None
            if count is None:
                raise ObjSemanticsError(f"{mtl_path}:{line_number}: unsupported texture option {option}")
            if cursor + count > len(tokens):
                raise ObjSemanticsError(f"{mtl_path}:{line_number}: incomplete texture option {option}")
            cursor += count
    if cursor == len(tokens):
        raise ObjSemanticsError(f"{mtl_path}:{line_number}: texture filename is missing")
    # Many exporters leave filenames containing spaces unquoted. Everything after
    # the options is the filename, as specified by the MTL texture-map syntax.
    texture_path = _local_path(" ".join(tokens[cursor:]), mtl_path.parent)
    if not texture_path.is_file():
        raise ObjSemanticsError(f"{mtl_path}:{line_number}: missing texture {texture_path}")
    indent = line[:len(line) - len(line.lstrip())]
    options = " ".join(tokens[:cursor])
    separator = f" {options}" if options else ""
    newline = "\r\n" if line.endswith("\r\n") else "\n" if line.endswith("\n") else ""
    # Absolute paths (including spaces) are accepted by the Wavefront map syntax.
    return f"{indent}{directive}{separator} {texture_path}{newline}"


def _read_material_library(path: Path) -> tuple[str, set[str]]:
    if not path.is_file():
        raise ObjSemanticsError(f"Missing material library: {path}")
    with path.open("r", encoding="utf-8", newline="") as stream:
        lines = stream.readlines()
    names: set[str] = set()
    rendered_lines = []
    for line_number, line in enumerate(lines, 1):
        directive, payload = _parts(line)
        if directive == "newmtl":
            name = " ".join(_tokens(payload, location=f"{path}:{line_number}"))
            if not name:
                raise ObjSemanticsError(f"{path}:{line_number}: material name is missing")
            names.add(name)
        if directive.startswith("map_") or directive in _TEXTURE_DIRECTIVES:
            line = _rewrite_texture(line, path, line_number)
        rendered_lines.append(line)
    return "".join(rendered_lines), names


def _validate_face(payload: str, declared: Counter, totals: Counter, location: str) -> None:
    corners = _tokens(payload, location=location)
    if len(corners) < 3:
        raise ObjSemanticsError(f"{location}: a face needs at least three vertices")
    for corner in corners:
        fields = corner.split("/")
        if len(fields) > 3 or not fields[0]:
            raise ObjSemanticsError(f"{location}: unsupported face reference {corner}")
        for index_text, kind in zip(fields, ("v", "vt", "vn")):
            if not index_text:
                continue
            try:
                index = int(index_text)
            except ValueError as exc:
                raise ObjSemanticsError(f"{location}: invalid face index {index_text}") from exc
            if index == 0 or (index > 0 and index > totals[kind]) or (index < 0 and -index > declared[kind]):
                raise ObjSemanticsError(f"{location}: face index {index} is outside {kind} records")


def split_obj_by_class(
    source: Path,
    output_dir: Path,
    material_to_class: dict[str, int],
) -> dict[int, Path]:
    """Write one OBJ per class and return ``{class_id: generated_path}``.

    Face lines and indices are byte-for-byte preserved. Every original face is
    written to exactly one output. Classes without faces produce no OBJ. Unknown
    materials, unsupported non-face geometry, missing textures and invalid indices
    raise ``ObjSemanticsError`` before output files are written. Existing output
    files are never overwritten. OBJ line continuations are explicitly unsupported.
    """
    source = Path(source).resolve()
    output_dir = Path(output_dir).resolve()
    if not source.is_file():
        raise ObjSemanticsError(f"Missing source OBJ: {source}")
    if source.is_relative_to(output_dir):
        raise ObjSemanticsError("The output directory must not contain the source OBJ")
    for material, class_id in material_to_class.items():
        if not isinstance(material, str) or not material.strip():
            raise ObjSemanticsError("Material names must be nonempty strings")
        if isinstance(class_id, bool) or not isinstance(class_id, int) or not 0 <= class_id <= 255:
            raise ObjSemanticsError(f"Class ID for {material} must be an integer in [0, 255]")
    with source.open("r", encoding="utf-8", newline="") as stream:
        lines = stream.readlines()
    totals = Counter(_parts(line)[0] for line in lines)
    declared = Counter()
    face_classes: dict[int, int] = {}
    libraries: dict[Path, tuple[Path, str]] = {}
    library_line_replacements: dict[int, str] = {}
    material_names: set[str] = set()
    used_materials: set[str] = set()
    current_material: str | None = None
    # A source name may contain whitespace; use a safe deterministic output prefix.
    prefix = re.sub(r"[^A-Za-z0-9_.-]+", "_", source.stem) or "mesh"
    for line_number, line in enumerate(lines, 1):
        directive, payload = _parts(line)
        if line.rstrip("\r\n").endswith("\\"):
            raise ObjSemanticsError(f"{source}:{line_number}: line continuations are unsupported")
        if directive in {"l", "p", "curv", "curv2", "surf"}:
            raise ObjSemanticsError(f"{source}:{line_number}: non-face geometry {directive} is unsupported")
        if directive in {"v", "vt", "vn"}:
            declared[directive] += 1
        elif directive == "mtllib":
            references = _tokens(payload, location=f"{source}:{line_number}")
            if not references:
                raise ObjSemanticsError(f"{source}:{line_number}: material library is missing")
            output_names = []
            for reference in references:
                original_mtl = _local_path(reference, source.parent)
                if original_mtl not in libraries:
                    contents, names = _read_material_library(original_mtl)
                    material_names.update(names)
                    destination = output_dir / f"{prefix}_semantic_materials_{len(libraries)}.mtl"
                    libraries[original_mtl] = (destination, contents)
                output_names.append(libraries[original_mtl][0].name)
            newline = "\r\n" if line.endswith("\r\n") else "\n" if line.endswith("\n") else ""
            library_line_replacements[line_number] = "mtllib " + " ".join(output_names) + newline
        elif directive == "usemtl":
            current_material = " ".join(_tokens(payload, location=f"{source}:{line_number}"))
            if current_material not in material_to_class:
                raise ObjSemanticsError(f"{source}:{line_number}: unmapped material {current_material!r}")
            used_materials.add(current_material)
        elif directive == "f":
            if current_material is None:
                raise ObjSemanticsError(f"{source}:{line_number}: face has no material")
            _validate_face(payload, declared, totals, f"{source}:{line_number}")
            face_classes[line_number] = material_to_class[current_material]
    if not face_classes:
        raise ObjSemanticsError(f"Source OBJ has no faces: {source}")
    if libraries and not used_materials.issubset(material_names):
        raise ObjSemanticsError(f"Materials missing from libraries: {sorted(used_materials - material_names)}")
    outputs = {
        class_id: output_dir / f"{prefix}_class_{class_id}.obj"
        for class_id in sorted(set(face_classes.values()))
    }
    destinations = [*outputs.values(), *(destination for destination, _ in libraries.values())]
    if len(set(destinations)) != len(destinations):
        raise ObjSemanticsError("Generated asset names collide")
    for destination in destinations:
        if destination.exists():
            raise FileExistsError(f"Refusing to overwrite generated asset: {destination}")
        if destination == source or destination in libraries:
            raise ObjSemanticsError(f"Generated asset would overwrite a source: {destination}")
    output_dir.mkdir(parents=True, exist_ok=True)
    created: list[Path] = []
    try:
        for destination, contents in libraries.values():
            with destination.open("x", encoding="utf-8", newline="") as stream:
                created.append(destination)
                stream.write(contents)
        for class_id, destination in outputs.items():
            with destination.open("x", encoding="utf-8", newline="") as stream:
                created.append(destination)
                for line_number, line in enumerate(lines, 1):
                    if line_number in face_classes and face_classes[line_number] != class_id:
                        continue
                    stream.write(library_line_replacements.get(line_number, line))
    except BaseException:
        for destination in created:
            destination.unlink(missing_ok=True)
        raise
    return outputs
