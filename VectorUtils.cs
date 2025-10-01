using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.Arm;
using System.Runtime.Intrinsics.Wasm;
using System.Runtime.Intrinsics.X86;
using System.Security.Claims;
using System.Text;
using System.Threading.Tasks;

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


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static RectangleF AsRectangleF(Vector128<float> v) => Unsafe.As<Vector128<float>, RectangleF>(ref v);

    // The microoptimizations needed to get this to run faster amount to 1uop on most hardware. Forget it.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float PerpDot(Vector128<float> left, Vector128<float> right) => Vector128.Dot(left, Vector128.Shuffle(right, Vector128.Create(1, 0, 2, 3)) * Vector128.Create(-1f, 1f, 0f, 0f));

    // For some reason using UnsafeAccessor doesnt work with GetElementUnsafe. whatever. Update these when either UnsafeAccessor works, or .NET gets changed here.
    /// <summary>
    /// Gets an Element from the Vector without argument checks. Only use this if you can guarantee an element in a slot.
    /// </summary>
    /// <typeparam name="T">Type of vector</typeparam>
    /// <param name="vector">Vector to retrieve value</param>
    /// <param name="index">Index to check</param>
    /// <remarks>Optimization notes: this runs in 1 instruction for integers, and 5 for floats. If you're only checking ==/!=, it may be better to use AsInt32 integers since integer and a bitwise check is much faster.</remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static T GetElementUnsafe<T>(in this Vector128<T> vector, int index) {
        Debug.Assert((index >= 0) && (index < Vector128<T>.Count));
        ref T address = ref Unsafe.As<Vector128<T>, T>(ref Unsafe.AsRef(in vector));
        return Unsafe.Add(ref address, index);
    }
    /// <summary>
    /// Sets an Element from the Vector without argument checks. Only use this if you can guarantee an element in a slot.
    /// </summary>
    /// <typeparam name="T">Type of Vector</typeparam>
    /// <param name="vector">Vector to change value in</param>
    /// <param name="index">Index to check</param>
    /// <param name="value">Value to set to</param>
    /// <remarks>Unlike GetElementUnsafe, SetElementUnsafe runs in the same number of instructions between ints and floats.</remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void SetElementUnsafe<T>(in this Vector128<T> vector, int index, T value) {
        Debug.Assert((index >= 0) && (index < Vector128<T>.Count));
        ref T address = ref Unsafe.As<Vector128<T>, T>(ref Unsafe.AsRef(in vector));
        Unsafe.Add(ref address, index) = value;
    }

    /// <summary>
    /// Attempts to do more optimal shuffling than Vector128 on *all* hardware, including nonAVX, for dynamic masks.<br/>If you know the order of the shuffle as a constant, don't use this.
    /// </summary>
    /// <param name="toBeShuffled">Vector to shuffle.</param>
    /// <param name="mask">Mask of permutations in order. Must contain 0/1/2/3 for each value.</param>
    /// <returns>A rearranged vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<float> DynamicShuffle(Vector128<float> toBeShuffled, Vector128<int> mask) {
        if (Avx.IsSupported) return Avx.PermuteVar(toBeShuffled, mask); // Easy!
        // Everything else here is significantly harder.
        // All of these do the same operation. Shuffling via bytes. 
        // For any given value of 0/1/2/3 integer, we need to create a Vector128<byte> with a corresponding value
        // 0          1          2          3
        // 0x03020100 0x07060504 0x0B0A0908 0x0F0E0D0C
        // Additional reminder: Fused-MultiplyAdd only works for floatingpoints up until AVX512-IFMA which, as of the time of writing, basically noone has lmao.
        else if (Ssse3.IsSupported) {
            mask = Vector128.Create(0x04040404) * mask + Vector128.Create(0x03020100);
            return Ssse3.Shuffle(toBeShuffled.AsByte(), mask.AsByte()).AsSingle();
        } else if (AdvSimd.Arm64.IsSupported) {
            mask = Vector128.Create(0x04040404) * mask + Vector128.Create(0x03020100);
            return AdvSimd.Arm64.VectorTableLookup(toBeShuffled.AsByte(), mask.AsByte()).AsSingle();
        } else if (PackedSimd.IsSupported) {
            mask = Vector128.Create(0x04040404) * mask + Vector128.Create(0x03020100);
            return PackedSimd.Swizzle(toBeShuffled.AsByte(), mask.AsByte()).AsSingle();
        } else { // Final case: we just do a manual shuffle.
            return Vector128.Shuffle(toBeShuffled, mask);
        }
    }

    #region Boring Stuff
    /// <summary>
    /// Attempts to do more optimal shuffling than Vector128 on *all* hardware, including nonAVX, for dynamic masks.<br/>If you know the order of the shuffle as a constant, don't use this.
    /// </summary>
    /// <param name="toBeShuffled">Vector to shuffle.</param>
    /// <param name="mask">Mask of permutations in order. Must contain 0/1/2/3 for each value.</param>
    /// <returns>A rearranged vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<int> DynamicShuffle(Vector128<int> toBeShuffled, Vector128<int> mask) => DynamicShuffle(toBeShuffled.AsSingle(), mask).AsInt32();
    /// <summary>
    /// Attempts to do more optimal shuffling than Vector128 on *all* hardware, including nonAVX, for dynamic masks.<br/>If you know the order of the shuffle as a constant, don't use this.
    /// </summary>
    /// <param name="toBeShuffled">Vector to shuffle.</param>
    /// <param name="mask">Mask of permutations in order. Must contain 0/1/2/3 for each value.</param>
    /// <returns>A rearranged vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<uint> DynamicShuffle(Vector128<uint> toBeShuffled, Vector128<int> mask) => DynamicShuffle(toBeShuffled.AsSingle(), mask).AsUInt32();
    /// <summary>
    /// Attempts to do more optimal shuffling than Vector128 on *all* hardware, including nonAVX, for dynamic masks.<br/>If you know the order of the shuffle as a constant, don't use this.
    /// </summary>
    /// <param name="toBeShuffled">Vector to shuffle.</param>
    /// <param name="mask">Mask of permutations in order. Must contain 0/1/2/3 for each value.</param>
    /// <returns>A rearranged vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<nint> DynamicShuffle(Vector128<nint> toBeShuffled, Vector128<int> mask) => DynamicShuffle(toBeShuffled.AsSingle(), mask).AsNInt();
    /// <summary>
    /// Attempts to do more optimal shuffling than Vector128 on *all* hardware, including nonAVX, for dynamic masks.<br/>If you know the order of the shuffle as a constant, don't use this.
    /// </summary>
    /// <param name="toBeShuffled">Vector to shuffle.</param>
    /// <param name="mask">Mask of permutations in order. Must contain 0/1/2/3 for each value.</param>
    /// <returns>A rearranged vector.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector128<nuint> DynamicShuffle(Vector128<nuint> toBeShuffled, Vector128<int> mask) => DynamicShuffle(toBeShuffled.AsSingle(), mask).AsNUInt();


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
    public static float PerpDot(Vector2 left, Vector2 right) => Vector128.Dot(left.AsVector128Unsafe(), Vector128.Shuffle(right.AsVector128Unsafe(), Vector128.Create(1, 0, 2, 3)) * Vector128.Create(-1f, 1f, 0f, 0f));
    #endregion
}

