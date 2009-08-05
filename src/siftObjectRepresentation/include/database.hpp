// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Authors: 2007 Bruno Damas
 *          2008 Dario Figueira
 *      Institute for Systems and Robotics (IST)
 *      IST, TULisbon, Portugal
 */


// OBS
/* when the current record is removed, the current record is assigned
to the previous record, if any exists, or to NULL otherwise. In both cases
the call to go_to_next() is guaranted to position the current record in the
record next to the one that has just been removed. */
/* insert_record: last_record pointer changes */
/* remove_record: last_record pointer changes,
   current goes to NULL */

#ifndef __Database__B
#define __Database__B

#include "common.hpp"

#include <iostream>
#include <list>
#include <algorithm>
using namespace std;

// CLASS INTERFACE


template <typename Type>
class Database {
    //      friend ostream &operator << <Type> (ostream &stream, Database<Type> const &);

public :
    Database( Database<Type> const &other );
    Database<Type> &operator=( Database<Type> const &other );
    Database( int num = 0, bool quiet_mode = true );
    ~Database();
    void reset( void );
    void reset( int new_n );
    void resize( int new_n );
    void shrink_size();

    int get_capacity() const;
    int get_size() const;
    bool is_full() const;
    bool is_empty() const;
    Type &operator[]( int ind );
    Type const &operator[]( int ind ) const;

    int get_first() const;
    int get_last() const;
    int get_current() const;
    int get_insertion_point() const;

    int go_to_next();
    int go_to_next(int record);
    int go_to_previous();
    int go_to_previous(int record);
    int go_to_first();
    int go_to_last();

    int insert_record( Type const &new_record );
    int replace_record( int record, Type const &new_record );
    int remove_record( int record );

    static void test_class();
    void data_dump() const;
    ostream &print_data(ostream &) const;

    // 	void sort();

private :

    void init( int n );
    void copy( Database<Type> const &other);
    void destroy();

    Type *data;
    int *index;
    int *backindex;

    int current;
    int size;
    int n;

    bool quiet;
};

template <typename Type>
ostream &operator << (ostream &stream, Database<Type> const &db) {
    return db.print_data( stream );
}


// Unfortunately it is necessary to keep the implementation and the interface
// of template classes on the same file !!

// CLASS IMPLEMENTATION


template <typename Type>
void Database<Type>::init( int num ) {
    size = current = 0;
    n = num;

    if( n <= 0 ) {
        n = 0;
        data = NULL;
        index = NULL;
        return;
    } else if( (( data = new Type[n] ) == NULL) ||
               (( index = new int[n]) == NULL) ||
               (( backindex = new int[n]) == NULL)) {
        cerr << "DataBase: Couldn't allocate memory. Quitting !!\n";
        exit(1);
    }

    for( int i = 0; i < n; i++)
        backindex[i] = index[i] = i;
}


template <typename Type>
void Database<Type>::copy( Database<Type> const &other) {
    current = other.current;
    size = other.size;
    n = other.n;
    quiet = other.quiet;

    if( n <= 0 ) {
        n = 0;
        data = NULL;
        index = NULL;
        return;
    } else if( (( data = new Type[n] ) == NULL) ||
               (( index = new int[n]) == NULL) ||
               (( backindex = new int[n]) == NULL)) {
        cerr << "DataBase: Couldn't allocate memory. Quitting !!\n";
        exit(1);
    }

    for(int i = 0; i < n; i++) {
        data[i] = other.data[i];
        index[i] = other.index[i];
        backindex[i] = other.backindex[i];
    }
}

template <typename Type>
void Database<Type>::destroy( void ) {
    if( n != 0 ) {
        delete[] data;
        delete[] index;
        delete[] backindex;
    }
}


template <typename Type>
Database<Type>::Database( Database<Type> const &other ) {
    copy(other);
}


template <typename Type>
Database<Type> &Database<Type>::operator=( Database<Type> const &other ) {
    if (this != &other) {
        destroy();
        copy(other);
    }

    return *this;
}


template <typename Type>
Database<Type>::Database( int num, bool quiet_mode ) {
    init(num);
    quiet = quiet_mode;
    if( NOT quiet )
        cout << "Database: Allocated memory for " << n << " records." << endl;
}

template <typename Type>
Database<Type>::~Database() {
    destroy();
    if( NOT quiet )
        cout << "Database: Memory deallocated." << endl;
}

template <typename Type>
void Database<Type>::reset( void ) {
    destroy();
    init(n);
    if( NOT quiet )
        cout << "Database: resetting memory." << endl;
}

template <typename Type>
void Database<Type>::reset( int new_n ) {
    destroy();
    init(new_n);
    if( NOT quiet )
        cout << "Database: resetting memory, " << n << " records reallocated." << endl;
}


template <typename Type>
void Database<Type>::resize( int new_n ) {
    if( new_n < size || new_n == n )
        return;
    else if( is_empty() ) {
        reset( new_n );
        return;
    }

    Type *data_tmp = data;
    int *index_tmp = index;
    int *backindex_tmp = backindex;

    if( (( data = new Type[new_n] ) == NULL) || (( index = new int[new_n]) == NULL) || (( backindex = new int[new_n]) == NULL) ) {
        cerr << "DataBase: Couldn't allocate memory. Quitting !!\n";
        exit(1);
    }

    int i;
    for( i = 0; i < size; i++)
        data[i] = data_tmp[index_tmp[i]];

    for( i = 0; i < new_n; i++)
        backindex[i] = index[i] = i;

    current = 0;
    n = new_n;

    delete[] data_tmp;
    delete[] index_tmp;
    delete[] backindex_tmp;

    if( NOT quiet )
        cout << "Database: resizing memory to " << n << " records." << endl;
}


template <typename Type>
inline void Database<Type>::shrink_size() {
    resize(size);
}


template <typename Type>
inline int Database<Type>::get_capacity() const {
    return n;
}

template <typename Type>
inline int Database<Type>::get_size() const {
    return size;
}

template <typename Type>
inline bool Database<Type>::is_full() const {
    return( size == n );
}

template <typename Type>
inline bool Database<Type>::is_empty() const {
    return( size == 0 );
}


template <typename Type>
inline Type &Database<Type>::operator[]( int ind ) {
    return data[ind];
}


template <typename Type>
inline Type const &Database<Type>::operator[]( int ind ) const {
    return data[ind];
}


template <typename Type>
inline int Database<Type>::get_first() const {
    return size == 0 ? -1 : index[0];
}

template <typename Type>
inline int Database<Type>::get_last() const {
    return size == 0 ? -1 : index[size-1];
}

template <typename Type>
inline int Database<Type>::get_current() const {
    return size == 0 ? -1 : index[current];
}

template <typename Type>
inline int Database<Type>::get_insertion_point() const {
    return n == 0 ? -1 : index[size];
}


template <typename Type>
int Database<Type>::go_to_first() {
    return size == 0 ? -1 : index[current = 0];
}

template <typename Type>
int Database<Type>::go_to_last() {
    return size == 0 ? -1 : index[current = size-1];
}

template <typename Type>
int Database<Type>::go_to_next() {
    return current >= size-1 ? -1 : index[++current];
}

template <typename Type>
int Database<Type>::go_to_next(int record) {
    return (record < 0 || record >= n || backindex[record] >= size-1) ?
           -1 : index[current = backindex[record] + 1];
}

template <typename Type>
int Database<Type>::go_to_previous() {
    return current == 0 ? -1 : index[--current];
}

template <typename Type>
int Database<Type>::go_to_previous(int record) {
    return (record < 0 || record >= n || backindex[record] >= size || backindex[record] == 0) ?
           -1 : index[current = backindex[record] - 1];
}

template <typename Type>
int Database<Type>::insert_record( Type const &new_record ) {
    if( size == n )
        return -1;

    data[index[size++]] = new_record;
    return index[size-1];
}


template <typename Type>
int Database<Type>::replace_record( int record, Type const &new_record ) {
    if( record < 0 || record >= n || backindex[record] >= size )
        return -1;
    else {
        data[record] = new_record;
        return record;
    }
}


template <typename Type>
int Database<Type>::remove_record( int record) {
    if( record < 0 || record >= n || backindex[record] >= size )
        return -1;
    else if( size == 1 ) {
        size--;
        return record;
    } else {
        index[backindex[record]] = index[size-1];	//Swap with last record
        backindex[index[size-1]] = backindex[record];
        index[size-1] = record;
        backindex[record] = size-1;

        size--;
        return record;
    }
}


template <typename Type>
void Database<Type>::data_dump() const {
    cout << "Database: listing internal registries." << endl;

    cout << "\tSize: " << size << endl;
    for( int i = 0; i < n; i++ )
    cout << "[" << i << "] -> " << index[i] << ", " << backindex[i] << ", " << data[i] << endl;
}

template <typename Type>
ostream &Database<Type>::print_data(ostream &stream) const {
    for( int i = 0; i < size; i++ )
        stream << "--Rec " << index[i] << "--" << endl << data[index[i]] << endl;

    stream << "   [" << size << " records]" << endl;

    return stream;
}


template <typename Type>
void Database<Type>::test_class() {
    int n;
    int np;
    Database<int> mem(4, false);

    n = 6;
    mem.insert_record(n);
    n = 7;
    mem.insert_record(n);
    n = 8;
    mem.insert_record(n);
    n = 9;
    mem.insert_record(n);
    n = 10;
    mem.insert_record(n);
    mem.data_dump();
    np = mem.get_first();
    mem.remove_record(np);
    mem.data_dump();
    n = 10;
    np = mem.get_last();
    mem.replace_record(np,n);
    mem.insert_record(n);
    mem.data_dump();
    np = mem.go_to_first();
    np = mem.go_to_next();
    mem.remove_record(np);
    mem.data_dump();
    cout << mem << endl;
    np = mem.go_to_first();
    mem.remove_record(np);
    mem.data_dump();
    np = mem.go_to_first();
    mem.remove_record(np);
    mem.data_dump();
    np = mem.go_to_first();
    mem.remove_record(np);
    mem.data_dump();

    n = 24;
    mem.insert_record(n);
    n = 44;
    mem.insert_record(n);
    mem.data_dump();
    mem.insert_record(33);
    cout << mem << endl;


    Database<int> mem2(10, false);
    mem2.insert_record(10);
    mem2.insert_record(20);
    mem2.insert_record(30);
    mem2.insert_record(40);
    mem2.insert_record(50);
    mem2.insert_record(60);
    mem2.insert_record(70);
    np = mem2.get_first();
    mem2.remove_record(np);
    np = mem2.get_first();
    mem2.remove_record(np);
    np = mem2.get_first();
    mem2.remove_record(np);

    cout << "Copying entire database. New database:" << endl;
    Database<int> new_mem;
    new_mem = mem2;
    cout << new_mem;
    cout << "Data DUMP:" << endl;
    new_mem.data_dump();

    new_mem.shrink_size();
    cout << "Shrinking new_database:" << endl;
    cout << new_mem;
    cout << "Data DUMP:" << endl;
    new_mem.data_dump();

    new_mem.resize(10);
    cout << "Resizing new_database:" << endl;
    cout << new_mem;
    cout << "Data DUMP:" << endl;
    new_mem.data_dump();


    cout << "Testing reset(5)" << endl;
    mem.reset(5);
    mem.data_dump();
}
// void sort()
// {
// 	int changes_made = 0;
// 	do
// 	{
// 		for(i=0; i<size; i++)
// 		{
// 			compare(data[index[i]],
// 		}
//
// 	}while(changes_made)
// }

#endif //__Database__B
/* EOF */
