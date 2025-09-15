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
    public float X; public float Y; private float radius; private float radiusSq;
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
    public Vector2 Position { [MethodImpl(MethodImplOptions.AggressiveInlining)] get => this.AsVector128().AsVector2(); }
    public float Radius {
        [MethodImpl(MethodImplOptions.AggressiveInlining)] get => radius;
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)] set { radius = value; radiusSq = value * value; }
    }
    public float RadiusSq { [MethodImpl(MethodImplOptions.AggressiveInlining)] get => radiusSq; }

    // Vectorized swappers

    [MethodImpl(MethodImplOptions.AggressiveInlining)] public static Circle AsCircle(Vector128<float> values) => Unsafe.As<Vector128<float>, Circle>(ref values);
    [MethodImpl(MethodImplOptions.AggressiveInlining)] public Vector128<float> AsVector128() => Unsafe.As<Circle, Vector128<float>>(ref this);
}
