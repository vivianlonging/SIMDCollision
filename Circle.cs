using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Text;
using System.Threading.Tasks;

namespace SIMDCollision; 
public struct Circle {
    public float X; public float Y; public float radius;
    public Circle(float x, float y, float radius) {
        X = x;
        Y = y;
        this.radius = radius;
    }
    public Circle(Vector2 position, float radius) {
        X = position.X;
        Y = position.Y;
        this.radius = radius;
    }
    public Vector2 Position { [MethodImpl(MethodImplOptions.AggressiveInlining)] get => AsVector128(this).AsVector2(); }
    public float Radius { [MethodImpl(MethodImplOptions.AggressiveInlining)] get => radius; }
    public float RadiusSq { [MethodImpl(MethodImplOptions.AggressiveInlining)] get => radius * radius; }

    // Vectorized swappers

    [MethodImpl(MethodImplOptions.AggressiveInlining)] public static Circle AsCircle(Vector128<float> values) => Unsafe.BitCast<Vector3, Circle>(values.AsVector3());
    [MethodImpl(MethodImplOptions.AggressiveInlining)] public static Vector128<float> AsVector128(Circle circle) => Unsafe.As<Circle, Vector128<float>>(ref circle);
}
