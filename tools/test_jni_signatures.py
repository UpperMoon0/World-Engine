"""Guard the Java/Rust primitive JNI call boundary without launching Minecraft."""
import re
import unittest
from pathlib import Path

PREFIX = "Java_com_nstut_worldengine_physics_rapier_Rapier3D_"
PRIMITIVES = dict(int="jint", long="jlong", double="jdouble", float="jfloat", boolean="jboolean")


def arguments(text):
    result, start, depth = [], 0, 0
    for index, char in enumerate(text):
        if char in "<([":
            depth += 1
        elif char in ">)]":
            depth -= 1
        elif char == "," and depth == 0:
            if text[start:index].strip():
                result.append(text[start:index].strip())
            start = index + 1
    if text[start:].strip():
        result.append(text[start:].strip())
    return result


def mismatches(java, rust):
    declarations = dict(re.findall(
        r"\bstatic\s+native\s+\w+(?:\[\])?\s+(\w+)\s*\(([^)]*)\)", java))
    exports = dict(re.findall(
        r"fn\s+" + PREFIX + r"(\w+)[^()]*\((.*?)\)\s*(?:->[^\{]+)?\{", rust, re.S))
    errors = []
    for name, params in declarations.items():
        if name not in exports:
            errors.append(f"{name}: missing Rust export")
            continue
        java_args, rust_args = arguments(params), arguments(exports[name])[2:]
        if len(java_args) != len(rust_args):
            errors.append(f"{name}: Java passes {len(java_args)} arguments, Rust reads {len(rust_args)}")
        for index, (java_arg, rust_arg) in enumerate(zip(java_args, rust_args)):
            java_type = re.sub(r"@\w+\s*|\bfinal\s+", "", java_arg).split()[0]
            rust_type = rust_arg.split(":", 1)[1].strip().split("::")[-1]
            if java_type in PRIMITIVES and rust_type != PRIMITIVES[java_type]:
                errors.append(f"{name} argument {index}: {java_type} versus {rust_type}")
    return declarations, errors


class JniSignatureTest(unittest.TestCase):
    def test_all_java_native_argument_counts_and_primitive_types_match_rust(self):
        root = Path(__file__).resolve().parents[1]
        java = (root / "worldengine_rapier/src/main/java/com/nstut/worldengine/physics/rapier/Rapier3D.java").read_text()
        rust = "\n".join(path.read_text() for path in
                         (root / "worldengine_rapier/src/main/rust/rapier/src").glob("*.rs"))
        declarations, errors = mismatches(java, rust)
        self.assertEqual(len(re.findall(r"\bstatic\s+native\b", java)), len(declarations))
        self.assertGreater(len(declarations), 0)
        self.assertEqual([], errors, "\n".join(errors))

    def test_detects_the_unpassed_dynamic_argument_and_primitive_mismatch(self):
        java = "private static native int newVoxelCollider(double friction, boolean fluid);"
        rust = f'pub extern "system" fn {PREFIX}newVoxelCollider(env: JNIEnv, cls: JClass, friction: jdouble, fluid: jboolean, dynamic: jboolean) -> jint {{}}'
        self.assertIn("Java passes 2 arguments, Rust reads 3", mismatches(java, rust)[1][0])
        rust = rust.replace(", dynamic: jboolean", "").replace("fluid: jboolean", "fluid: jint")
        self.assertIn("boolean versus jint", mismatches(java, rust)[1][0])
        rust = rust.replace("fluid: jint", "fluid: jni::sys::jboolean")
        self.assertEqual([], mismatches(java, rust)[1])


if __name__ == "__main__":
    unittest.main()
