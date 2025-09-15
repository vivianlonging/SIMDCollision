﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.Arm;
using System.Runtime.Intrinsics.X86;
using System.Runtime.Intrinsics.Wasm;
using System.Text;
using System.Threading.Tasks;
using System.Diagnostics.CodeAnalysis;

namespace SIMDCollision;

/// <summary>
/// Defines an Edge Collision Ruleset. Exclusions are relative to Rectangle B, so "LeftInclusion" means that A.Right == B.Left returns true.
/// </summary>
[Flags]
public enum EdgeCollisionRule {
    /// <summary>
    /// All edge collisions return false.
    /// </summary>
    NoEdgesIn = 0,
    /// <summary>
    /// A.Right == B.Left returns true.
    /// </summary>
    LeftInclusion = 1,
    /// <summary>
    /// A.Bottom == B.Top returns true.
    /// </summary>
    TopInclusion = 2,
    /// <summary>
    /// A.Left == B.Right returns true
    /// </summary>
    RightInclusion = 4,
    /// <summary>
    /// A.Top == B.Bottom returns true
    /// </summary>
    BottomInclusion = 8,
    /// <summary>
    /// All edge detection. "Near" side is within the rectangle, "Far" side is as well.
    /// </summary>
    AllEdgesIn = 15
}

public static class VectorUtils {

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<int> RuleToMask(EdgeCollisionRule rule) => ~Vector128.Equals(Vector128.Create((int)rule) & Vector128.Create(1, 2, 4, 8), Vector128<int>.Zero);


    /// <summary>
    /// Loads the Vector2 as left 128bit vector in the format that collisions use.
    /// </summary>
    /// <param name="self">Vector2</param>
    /// <returns>Vector128 used for vectorized collisions.</returns>
    // x86: vmovddup xmm0, [esp+4]
    //      vxorps xmm0, xmm0, [VectorUtils.InCollisionForm(System.Numerics.Vector2)]
    //      vmovups [ecx], xmm0
    //      ret 8
    // ARM: (estimate)
    //      mov         r1, ???
    //      LD1R        Vt.2D r1
    //      EOR         r1 r1 Const
    //      Further instructions to return float32x4 in r0
    // Const: 0x7FFFFFF | 0x7FFFFFFF | 0x00000000 | 0x00000000
    // No fast WASM support yet :(
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<float> InCollisionForm(Vector2 self) {
        return Vector128.Create(Unsafe.BitCast<Vector2, double>(self)).AsSingle() ^ Vector128.Create(0, 0, 1 << 31, 1 << 31).AsSingle();
    }


    /// <summary>
    /// Use this *only* with structs with two floats in the values, e.g. Microsoft.Xna.Framework.Vector2.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static T As<T>(Vector2 vector) => Unsafe.As<Vector2, T>(ref vector);
    /// <summary>
    /// Use this *only* with structs with three floats in the values, e.g. Microsoft.Xna.Framework.Vector3.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static T As<T>(Vector3 vector) => Unsafe.As<Vector3, T>(ref vector);
    /// <summary>
    /// Use this *only* with structs with four floats in the values, e.g. Microsoft.Xna.Framework.Vector4.
    /// </summary>
    // This could also be used with Plane and Quaternion, but it doesn't work nicely, just trust me on that.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static T As<T>(Vector4 vector) => Unsafe.As<Vector4, T>(ref vector);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static RectangleF AsRectangleF(Vector128<float> v) => Unsafe.As<Vector128<float>, RectangleF>(ref v);

    // While this can be optimized further, I've chosen not to as the code complexity would grow significantly and be dependent on architecture.
    // If you already know what architecture you're optimizing for, why are you using C# :3
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float PerpDot(Vector128<float> left, Vector128<float> right) => Vector128.Dot(left, Vector128.Shuffle(right, Vector128.Create(1,0,2,3)) * Vector128.Create(-1f, 1f, 0f, 0f));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float PerpDot(Vector2 left, Vector2 right) => Vector128.Dot(left.AsVector128Unsafe(), Vector128.Shuffle(right.AsVector128Unsafe(), Vector128.Create(1, 0, 2, 3)) * Vector128.Create(-1f, 1f, 0f, 0f));
}

