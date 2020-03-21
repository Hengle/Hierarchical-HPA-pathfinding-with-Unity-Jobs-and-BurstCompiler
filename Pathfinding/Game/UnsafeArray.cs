using System;
using Unity.Assertions;
using Unity.Collections.LowLevel.Unsafe;

public unsafe struct UnsafeArray : IDisposable
{
    [NativeDisableUnsafePtrRestriction]
    void* _ptr;
    Int64 _refs;
    Int32 _length;
    IntPtr _typeHandle;

    public Int32 Length
    {
        get
        {
            return _length;
        }
    }

    public UnsafeArray( Type type , Int32 length )
    {
        //Assert.Check( UnsafeUtility.IsBlittable( type ) );

        // byte size of the memory we need
        var byteSize = length * UnsafeUtility.SizeOf( type );

        _refs = 0;
        _length = length;
        _typeHandle = type.TypeHandle.Value;

        // allocate memory
        _ptr = UnsafeUtility.Malloc( byteSize , 4 , Unity.Collections.Allocator.Persistent );

        // clear memory
        UnsafeUtility.MemClear( _ptr , byteSize );
    }

    public static UnsafeArray Create<T>( Int32 length )
    {
        return new UnsafeArray( typeof( T ) , length );
    }

    public void Dispose()
    {
        //Assert.Check( _refs == 0 );

        if ( _refs != 0 )
        {
            throw new InvalidOperationException( "All refs have not been released" );
        }

        if ( _ptr != null )
        {
            // free memory
            UnsafeUtility.Free( _ptr , Unity.Collections.Allocator.Persistent );

            // clear pointer
            _ptr = null;
            _length = 0;
            _typeHandle = default( IntPtr );
        }
    }

    public T Get<T>( Int32 index ) where T : struct
    {
        //Assert.Check( typeof( T ).TypeHandle.Value == _typeHandle );

        if ( index >= 0 && index < _length )
        {
            return UnsafeUtility.ReadArrayElement<T>( _ptr , index );
        }
        else
        {
            throw new IndexOutOfRangeException();
        }
    }

    public void Set<T>( Int32 index , T value ) where T : struct
    {
        //Assert.Check( typeof( T ).TypeHandle.Value == _typeHandle );

        if ( index >= 0 && index < _length )
        {
            UnsafeUtility.WriteArrayElement<T>( _ptr , index , value );
        }
        else
        {
            throw new IndexOutOfRangeException();
        }
    }

    public UnsafeArrayRef AllocRef( Int32 refIndex )
    {
        //Assert.Check( refIndex != 0 );

        if ( ( _refs & ( 1L << refIndex ) ) == 0 )
        {
            // claim reference index
            _refs |= ( 1L << refIndex );

            // return reference
            return new UnsafeArrayRef( _ptr , _length , _typeHandle , refIndex );
        }

        throw new InvalidOperationException( String.Format( "Reference {0} has already been allocated" , refIndex ) );
    }

    public UnsafeArrayRef AllocRef()
    {
        for ( Int32 i = 1; i < 64; ++i )
        {
            if ( ( _refs & ( 1L << i ) ) == 0 )
            {
                return AllocRef( i );
            }
        }

        throw new InvalidOperationException( "All 64 refs have been allocated" );
    }

    public void ReleaseRef( ref UnsafeArrayRef reference )
    {
        if ( ( _refs & ( 1L << reference.Index ) ) != 0 )
        {
            // release reference
            _refs &= ~( 1L << reference.Index );

            // clear reference also
            reference = default( UnsafeArrayRef );
        }
    }
}

public unsafe struct UnsafeArrayRef
{
    [NativeDisableUnsafePtrRestriction]
    void* _ptr;
    Int32 _length;
    Int32 _refIndex;
    IntPtr _typeHandle;

    public Int32 Length
    {
        get
        {
            return _length;
        }
    }

    public Int32 Index
    {
        get
        {
            return _refIndex;
        }
    }

    public Boolean IsValid
    {
        get
        {
            return _ptr != null && _refIndex != 0;
        }
    }

    public UnsafeArrayRef( void* ptr , Int32 length , IntPtr typeHandle , Int32 refIndex )
    {
        _ptr = ptr;
        _length = length;
        _typeHandle = typeHandle;
        _refIndex = refIndex;
    }

    public T Get<T>( Int32 index ) where T : struct
    {
        //Assert.Check( typeof( T ).TypeHandle.Value == _typeHandle );

        if ( index >= 0 && index < _length )
        {
            return UnsafeUtility.ReadArrayElement<T>( _ptr , index );
        }
        else
        {
            throw new IndexOutOfRangeException();
        }
    }

    public void Set<T>( Int32 index , T value ) where T : struct
    {
        //Assert.Check( typeof( T ).TypeHandle.Value == _typeHandle );

        if ( index >= 0 && index < _length )
        {
            UnsafeUtility.WriteArrayElement<T>( _ptr , index , value );
        }
        else
        {
            throw new IndexOutOfRangeException();
        }
    }
}
