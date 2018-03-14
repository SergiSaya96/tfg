// Copyright (c) 2005-2014 Code Synthesis Tools CC
//
// This program was generated by CodeSynthesis XSD, an XML Schema to
// C++ data binding compiler.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
//
// In addition, as a special exception, Code Synthesis Tools CC gives
// permission to link this program with the Xerces-C++ library (or with
// modified versions of Xerces-C++ that use the same license as Xerces-C++),
// and distribute linked combinations including the two. You must obey
// the GNU General Public License version 2 in all respects for all of
// the code used other than Xerces-C++. If you modify this copy of the
// program, you may extend this exception to your version of the program,
// but you are not obligated to do so. If you do not wish to do so, delete
// this exception statement from your version.
//
// Furthermore, Code Synthesis Tools CC makes a special exception for
// the Free/Libre and Open Source Software (FLOSS) which is described
// in the accompanying FLOSSE file.
//

#ifndef DYNAMIXEL_MOTOR_CFG_FILE_HXX
#define DYNAMIXEL_MOTOR_CFG_FILE_HXX

#ifndef XSD_USE_CHAR
#define XSD_USE_CHAR
#endif

#ifndef XSD_CXX_TREE_USE_CHAR
#define XSD_CXX_TREE_USE_CHAR
#endif

// Begin prologue.
//
//
// End prologue.

#include <xsd/cxx/config.hxx>

#if (XSD_INT_VERSION != 4000000L)
#error XSD runtime version mismatch
#endif

#include <xsd/cxx/pre.hxx>

#include <xsd/cxx/xml/char-utf8.hxx>

#include <xsd/cxx/tree/exceptions.hxx>
#include <xsd/cxx/tree/elements.hxx>
#include <xsd/cxx/tree/types.hxx>

#include <xsd/cxx/xml/error-handler.hxx>

#include <xsd/cxx/xml/dom/auto-ptr.hxx>

#include <xsd/cxx/tree/parsing.hxx>
#include <xsd/cxx/tree/parsing/byte.hxx>
#include <xsd/cxx/tree/parsing/unsigned-byte.hxx>
#include <xsd/cxx/tree/parsing/short.hxx>
#include <xsd/cxx/tree/parsing/unsigned-short.hxx>
#include <xsd/cxx/tree/parsing/int.hxx>
#include <xsd/cxx/tree/parsing/unsigned-int.hxx>
#include <xsd/cxx/tree/parsing/long.hxx>
#include <xsd/cxx/tree/parsing/unsigned-long.hxx>
#include <xsd/cxx/tree/parsing/boolean.hxx>
#include <xsd/cxx/tree/parsing/float.hxx>
#include <xsd/cxx/tree/parsing/double.hxx>
#include <xsd/cxx/tree/parsing/decimal.hxx>

#include <xsd/cxx/xml/dom/serialization-header.hxx>
#include <xsd/cxx/tree/serialization.hxx>
#include <xsd/cxx/tree/serialization/byte.hxx>
#include <xsd/cxx/tree/serialization/unsigned-byte.hxx>
#include <xsd/cxx/tree/serialization/short.hxx>
#include <xsd/cxx/tree/serialization/unsigned-short.hxx>
#include <xsd/cxx/tree/serialization/int.hxx>
#include <xsd/cxx/tree/serialization/unsigned-int.hxx>
#include <xsd/cxx/tree/serialization/long.hxx>
#include <xsd/cxx/tree/serialization/unsigned-long.hxx>
#include <xsd/cxx/tree/serialization/boolean.hxx>
#include <xsd/cxx/tree/serialization/float.hxx>
#include <xsd/cxx/tree/serialization/double.hxx>
#include <xsd/cxx/tree/serialization/decimal.hxx>

namespace xml_schema
{
  // anyType and anySimpleType.
  //
  typedef ::xsd::cxx::tree::type type;
  typedef ::xsd::cxx::tree::simple_type< char, type > simple_type;
  typedef ::xsd::cxx::tree::type container;

  // 8-bit
  //
  typedef signed char byte;
  typedef unsigned char unsigned_byte;

  // 16-bit
  //
  typedef short short_;
  typedef unsigned short unsigned_short;

  // 32-bit
  //
  typedef int int_;
  typedef unsigned int unsigned_int;

  // 64-bit
  //
  typedef long long long_;
  typedef unsigned long long unsigned_long;

  // Supposed to be arbitrary-length integral types.
  //
  typedef long long integer;
  typedef long long non_positive_integer;
  typedef unsigned long long non_negative_integer;
  typedef unsigned long long positive_integer;
  typedef long long negative_integer;

  // Boolean.
  //
  typedef bool boolean;

  // Floating-point types.
  //
  typedef float float_;
  typedef double double_;
  typedef double decimal;

  // String types.
  //
  typedef ::xsd::cxx::tree::string< char, simple_type > string;
  typedef ::xsd::cxx::tree::normalized_string< char, string > normalized_string;
  typedef ::xsd::cxx::tree::token< char, normalized_string > token;
  typedef ::xsd::cxx::tree::name< char, token > name;
  typedef ::xsd::cxx::tree::nmtoken< char, token > nmtoken;
  typedef ::xsd::cxx::tree::nmtokens< char, simple_type, nmtoken > nmtokens;
  typedef ::xsd::cxx::tree::ncname< char, name > ncname;
  typedef ::xsd::cxx::tree::language< char, token > language;

  // ID/IDREF.
  //
  typedef ::xsd::cxx::tree::id< char, ncname > id;
  typedef ::xsd::cxx::tree::idref< char, ncname, type > idref;
  typedef ::xsd::cxx::tree::idrefs< char, simple_type, idref > idrefs;

  // URI.
  //
  typedef ::xsd::cxx::tree::uri< char, simple_type > uri;

  // Qualified name.
  //
  typedef ::xsd::cxx::tree::qname< char, simple_type, uri, ncname > qname;

  // Binary.
  //
  typedef ::xsd::cxx::tree::buffer< char > buffer;
  typedef ::xsd::cxx::tree::base64_binary< char, simple_type > base64_binary;
  typedef ::xsd::cxx::tree::hex_binary< char, simple_type > hex_binary;

  // Date/time.
  //
  typedef ::xsd::cxx::tree::time_zone time_zone;
  typedef ::xsd::cxx::tree::date< char, simple_type > date;
  typedef ::xsd::cxx::tree::date_time< char, simple_type > date_time;
  typedef ::xsd::cxx::tree::duration< char, simple_type > duration;
  typedef ::xsd::cxx::tree::gday< char, simple_type > gday;
  typedef ::xsd::cxx::tree::gmonth< char, simple_type > gmonth;
  typedef ::xsd::cxx::tree::gmonth_day< char, simple_type > gmonth_day;
  typedef ::xsd::cxx::tree::gyear< char, simple_type > gyear;
  typedef ::xsd::cxx::tree::gyear_month< char, simple_type > gyear_month;
  typedef ::xsd::cxx::tree::time< char, simple_type > time;

  // Entity.
  //
  typedef ::xsd::cxx::tree::entity< char, ncname > entity;
  typedef ::xsd::cxx::tree::entities< char, simple_type, entity > entities;

  typedef ::xsd::cxx::tree::content_order content_order;
  // Namespace information and list stream. Used in
  // serialization functions.
  //
  typedef ::xsd::cxx::xml::dom::namespace_info< char > namespace_info;
  typedef ::xsd::cxx::xml::dom::namespace_infomap< char > namespace_infomap;
  typedef ::xsd::cxx::tree::list_stream< char > list_stream;
  typedef ::xsd::cxx::tree::as_double< double_ > as_double;
  typedef ::xsd::cxx::tree::as_decimal< decimal > as_decimal;
  typedef ::xsd::cxx::tree::facet facet;

  // Flags and properties.
  //
  typedef ::xsd::cxx::tree::flags flags;
  typedef ::xsd::cxx::tree::properties< char > properties;

  // Parsing/serialization diagnostics.
  //
  typedef ::xsd::cxx::tree::severity severity;
  typedef ::xsd::cxx::tree::error< char > error;
  typedef ::xsd::cxx::tree::diagnostics< char > diagnostics;

  // Exceptions.
  //
  typedef ::xsd::cxx::tree::exception< char > exception;
  typedef ::xsd::cxx::tree::bounds< char > bounds;
  typedef ::xsd::cxx::tree::duplicate_id< char > duplicate_id;
  typedef ::xsd::cxx::tree::parsing< char > parsing;
  typedef ::xsd::cxx::tree::expected_element< char > expected_element;
  typedef ::xsd::cxx::tree::unexpected_element< char > unexpected_element;
  typedef ::xsd::cxx::tree::expected_attribute< char > expected_attribute;
  typedef ::xsd::cxx::tree::unexpected_enumerator< char > unexpected_enumerator;
  typedef ::xsd::cxx::tree::expected_text_content< char > expected_text_content;
  typedef ::xsd::cxx::tree::no_prefix_mapping< char > no_prefix_mapping;
  typedef ::xsd::cxx::tree::serialization< char > serialization;

  // Error handler callback interface.
  //
  typedef ::xsd::cxx::xml::error_handler< char > error_handler;

  // DOM interaction.
  //
  namespace dom
  {
    // Automatic pointer for DOMDocument.
    //
    using ::xsd::cxx::xml::dom::auto_ptr;

#ifndef XSD_CXX_TREE_TREE_NODE_KEY__XML_SCHEMA
#define XSD_CXX_TREE_TREE_NODE_KEY__XML_SCHEMA
    // DOM user data key for back pointers to tree nodes.
    //
    const XMLCh* const tree_node_key = ::xsd::cxx::tree::user_data_keys::node;
#endif
  }
}

// Forward declarations.
//
class alarm_t;
class dynamixel_motor_config_t;

#include <memory>    // ::std::auto_ptr
#include <limits>    // std::numeric_limits
#include <algorithm> // std::binary_search

#include <xsd/cxx/xml/char-utf8.hxx>

#include <xsd/cxx/tree/exceptions.hxx>
#include <xsd/cxx/tree/elements.hxx>
#include <xsd/cxx/tree/containers.hxx>
#include <xsd/cxx/tree/list.hxx>

#include <xsd/cxx/xml/dom/parsing-header.hxx>

class alarm_t: public ::xml_schema::string
{
  public:
  enum value
  {
    instruction_error,
    overload_error,
    checksum_error,
    range_error,
    overheating_error,
    angle_limit_error,
    input_voltage_error
  };

  alarm_t (value v);

  alarm_t (const char* v);

  alarm_t (const ::std::string& v);

  alarm_t (const ::xml_schema::string& v);

  alarm_t (const ::xercesc::DOMElement& e,
           ::xml_schema::flags f = 0,
           ::xml_schema::container* c = 0);

  alarm_t (const ::xercesc::DOMAttr& a,
           ::xml_schema::flags f = 0,
           ::xml_schema::container* c = 0);

  alarm_t (const ::std::string& s,
           const ::xercesc::DOMElement* e,
           ::xml_schema::flags f = 0,
           ::xml_schema::container* c = 0);

  alarm_t (const alarm_t& x,
           ::xml_schema::flags f = 0,
           ::xml_schema::container* c = 0);

  virtual alarm_t*
  _clone (::xml_schema::flags f = 0,
          ::xml_schema::container* c = 0) const;

  alarm_t&
  operator= (value v);

  virtual
  operator value () const
  {
    return _xsd_alarm_t_convert ();
  }

  protected:
  value
  _xsd_alarm_t_convert () const;

  public:
  static const char* const _xsd_alarm_t_literals_[7];
  static const value _xsd_alarm_t_indexes_[7];
};

class dynamixel_motor_config_t: public ::xml_schema::type
{
  public:
  // alarm_shtdwn
  //
  typedef ::xml_schema::unsigned_byte alarm_shtdwn_type;
  typedef ::xsd::cxx::tree::traits< alarm_shtdwn_type, char > alarm_shtdwn_traits;

  const alarm_shtdwn_type&
  alarm_shtdwn () const;

  alarm_shtdwn_type&
  alarm_shtdwn ();

  void
  alarm_shtdwn (const alarm_shtdwn_type& x);

  // max_angle
  //
  typedef ::xml_schema::float_ max_angle_type;
  typedef ::xsd::cxx::tree::traits< max_angle_type, char > max_angle_traits;

  const max_angle_type&
  max_angle () const;

  max_angle_type&
  max_angle ();

  void
  max_angle (const max_angle_type& x);

  // min_angle
  //
  typedef ::xml_schema::float_ min_angle_type;
  typedef ::xsd::cxx::tree::traits< min_angle_type, char > min_angle_traits;

  const min_angle_type&
  min_angle () const;

  min_angle_type&
  min_angle ();

  void
  min_angle (const min_angle_type& x);

  // temp_limit
  //
  typedef ::xml_schema::int_ temp_limit_type;
  typedef ::xsd::cxx::tree::traits< temp_limit_type, char > temp_limit_traits;

  const temp_limit_type&
  temp_limit () const;

  temp_limit_type&
  temp_limit ();

  void
  temp_limit (const temp_limit_type& x);

  // max_voltage
  //
  typedef ::xml_schema::float_ max_voltage_type;
  typedef ::xsd::cxx::tree::traits< max_voltage_type, char > max_voltage_traits;

  const max_voltage_type&
  max_voltage () const;

  max_voltage_type&
  max_voltage ();

  void
  max_voltage (const max_voltage_type& x);

  // min_voltage
  //
  typedef ::xml_schema::float_ min_voltage_type;
  typedef ::xsd::cxx::tree::traits< min_voltage_type, char > min_voltage_traits;

  const min_voltage_type&
  min_voltage () const;

  min_voltage_type&
  min_voltage ();

  void
  min_voltage (const min_voltage_type& x);

  // max_torque
  //
  typedef ::xml_schema::float_ max_torque_type;
  typedef ::xsd::cxx::tree::traits< max_torque_type, char > max_torque_traits;

  const max_torque_type&
  max_torque () const;

  max_torque_type&
  max_torque ();

  void
  max_torque (const max_torque_type& x);

  // cw_comp_margin
  //
  typedef ::xml_schema::unsigned_byte cw_comp_margin_type;
  typedef ::xsd::cxx::tree::traits< cw_comp_margin_type, char > cw_comp_margin_traits;

  const cw_comp_margin_type&
  cw_comp_margin () const;

  cw_comp_margin_type&
  cw_comp_margin ();

  void
  cw_comp_margin (const cw_comp_margin_type& x);

  // ccw_comp_margin
  //
  typedef ::xml_schema::unsigned_byte ccw_comp_margin_type;
  typedef ::xsd::cxx::tree::traits< ccw_comp_margin_type, char > ccw_comp_margin_traits;

  const ccw_comp_margin_type&
  ccw_comp_margin () const;

  ccw_comp_margin_type&
  ccw_comp_margin ();

  void
  ccw_comp_margin (const ccw_comp_margin_type& x);

  // cw_comp_slope
  //
  typedef ::xml_schema::unsigned_byte cw_comp_slope_type;
  typedef ::xsd::cxx::tree::traits< cw_comp_slope_type, char > cw_comp_slope_traits;

  const cw_comp_slope_type&
  cw_comp_slope () const;

  cw_comp_slope_type&
  cw_comp_slope ();

  void
  cw_comp_slope (const cw_comp_slope_type& x);

  // ccw_comp_slope
  //
  typedef ::xml_schema::unsigned_byte ccw_comp_slope_type;
  typedef ::xsd::cxx::tree::traits< ccw_comp_slope_type, char > ccw_comp_slope_traits;

  const ccw_comp_slope_type&
  ccw_comp_slope () const;

  ccw_comp_slope_type&
  ccw_comp_slope ();

  void
  ccw_comp_slope (const ccw_comp_slope_type& x);

  // punch
  //
  typedef ::xml_schema::unsigned_byte punch_type;
  typedef ::xsd::cxx::tree::traits< punch_type, char > punch_traits;

  const punch_type&
  punch () const;

  punch_type&
  punch ();

  void
  punch (const punch_type& x);

  // kp
  //
  typedef ::xml_schema::unsigned_byte kp_type;
  typedef ::xsd::cxx::tree::traits< kp_type, char > kp_traits;

  const kp_type&
  kp () const;

  kp_type&
  kp ();

  void
  kp (const kp_type& x);

  // ki
  //
  typedef ::xml_schema::unsigned_byte ki_type;
  typedef ::xsd::cxx::tree::traits< ki_type, char > ki_traits;

  const ki_type&
  ki () const;

  ki_type&
  ki ();

  void
  ki (const ki_type& x);

  // kd
  //
  typedef ::xml_schema::unsigned_byte kd_type;
  typedef ::xsd::cxx::tree::traits< kd_type, char > kd_traits;

  const kd_type&
  kd () const;

  kd_type&
  kd ();

  void
  kd (const kd_type& x);

  // Constructors.
  //
  dynamixel_motor_config_t (const alarm_shtdwn_type&,
                            const max_angle_type&,
                            const min_angle_type&,
                            const temp_limit_type&,
                            const max_voltage_type&,
                            const min_voltage_type&,
                            const max_torque_type&,
                            const cw_comp_margin_type&,
                            const ccw_comp_margin_type&,
                            const cw_comp_slope_type&,
                            const ccw_comp_slope_type&,
                            const punch_type&,
                            const kp_type&,
                            const ki_type&,
                            const kd_type&);

  dynamixel_motor_config_t (const ::xercesc::DOMElement& e,
                            ::xml_schema::flags f = 0,
                            ::xml_schema::container* c = 0);

  dynamixel_motor_config_t (const dynamixel_motor_config_t& x,
                            ::xml_schema::flags f = 0,
                            ::xml_schema::container* c = 0);

  virtual dynamixel_motor_config_t*
  _clone (::xml_schema::flags f = 0,
          ::xml_schema::container* c = 0) const;

  dynamixel_motor_config_t&
  operator= (const dynamixel_motor_config_t& x);

  virtual 
  ~dynamixel_motor_config_t ();

  // Implementation.
  //
  protected:
  void
  parse (::xsd::cxx::xml::dom::parser< char >&,
         ::xml_schema::flags);

  protected:
  ::xsd::cxx::tree::one< alarm_shtdwn_type > alarm_shtdwn_;
  ::xsd::cxx::tree::one< max_angle_type > max_angle_;
  ::xsd::cxx::tree::one< min_angle_type > min_angle_;
  ::xsd::cxx::tree::one< temp_limit_type > temp_limit_;
  ::xsd::cxx::tree::one< max_voltage_type > max_voltage_;
  ::xsd::cxx::tree::one< min_voltage_type > min_voltage_;
  ::xsd::cxx::tree::one< max_torque_type > max_torque_;
  ::xsd::cxx::tree::one< cw_comp_margin_type > cw_comp_margin_;
  ::xsd::cxx::tree::one< ccw_comp_margin_type > ccw_comp_margin_;
  ::xsd::cxx::tree::one< cw_comp_slope_type > cw_comp_slope_;
  ::xsd::cxx::tree::one< ccw_comp_slope_type > ccw_comp_slope_;
  ::xsd::cxx::tree::one< punch_type > punch_;
  ::xsd::cxx::tree::one< kp_type > kp_;
  ::xsd::cxx::tree::one< ki_type > ki_;
  ::xsd::cxx::tree::one< kd_type > kd_;
};

#include <iosfwd>

#include <xercesc/sax/InputSource.hpp>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMErrorHandler.hpp>

// Parse a URI or a local file.
//

::std::auto_ptr< ::dynamixel_motor_config_t >
dynamixel_motor_config (const ::std::string& uri,
                        ::xml_schema::flags f = 0,
                        const ::xml_schema::properties& p = ::xml_schema::properties ());

::std::auto_ptr< ::dynamixel_motor_config_t >
dynamixel_motor_config (const ::std::string& uri,
                        ::xml_schema::error_handler& eh,
                        ::xml_schema::flags f = 0,
                        const ::xml_schema::properties& p = ::xml_schema::properties ());

::std::auto_ptr< ::dynamixel_motor_config_t >
dynamixel_motor_config (const ::std::string& uri,
                        ::xercesc::DOMErrorHandler& eh,
                        ::xml_schema::flags f = 0,
                        const ::xml_schema::properties& p = ::xml_schema::properties ());

// Parse std::istream.
//

::std::auto_ptr< ::dynamixel_motor_config_t >
dynamixel_motor_config (::std::istream& is,
                        ::xml_schema::flags f = 0,
                        const ::xml_schema::properties& p = ::xml_schema::properties ());

::std::auto_ptr< ::dynamixel_motor_config_t >
dynamixel_motor_config (::std::istream& is,
                        ::xml_schema::error_handler& eh,
                        ::xml_schema::flags f = 0,
                        const ::xml_schema::properties& p = ::xml_schema::properties ());

::std::auto_ptr< ::dynamixel_motor_config_t >
dynamixel_motor_config (::std::istream& is,
                        ::xercesc::DOMErrorHandler& eh,
                        ::xml_schema::flags f = 0,
                        const ::xml_schema::properties& p = ::xml_schema::properties ());

::std::auto_ptr< ::dynamixel_motor_config_t >
dynamixel_motor_config (::std::istream& is,
                        const ::std::string& id,
                        ::xml_schema::flags f = 0,
                        const ::xml_schema::properties& p = ::xml_schema::properties ());

::std::auto_ptr< ::dynamixel_motor_config_t >
dynamixel_motor_config (::std::istream& is,
                        const ::std::string& id,
                        ::xml_schema::error_handler& eh,
                        ::xml_schema::flags f = 0,
                        const ::xml_schema::properties& p = ::xml_schema::properties ());

::std::auto_ptr< ::dynamixel_motor_config_t >
dynamixel_motor_config (::std::istream& is,
                        const ::std::string& id,
                        ::xercesc::DOMErrorHandler& eh,
                        ::xml_schema::flags f = 0,
                        const ::xml_schema::properties& p = ::xml_schema::properties ());

// Parse xercesc::InputSource.
//

::std::auto_ptr< ::dynamixel_motor_config_t >
dynamixel_motor_config (::xercesc::InputSource& is,
                        ::xml_schema::flags f = 0,
                        const ::xml_schema::properties& p = ::xml_schema::properties ());

::std::auto_ptr< ::dynamixel_motor_config_t >
dynamixel_motor_config (::xercesc::InputSource& is,
                        ::xml_schema::error_handler& eh,
                        ::xml_schema::flags f = 0,
                        const ::xml_schema::properties& p = ::xml_schema::properties ());

::std::auto_ptr< ::dynamixel_motor_config_t >
dynamixel_motor_config (::xercesc::InputSource& is,
                        ::xercesc::DOMErrorHandler& eh,
                        ::xml_schema::flags f = 0,
                        const ::xml_schema::properties& p = ::xml_schema::properties ());

// Parse xercesc::DOMDocument.
//

::std::auto_ptr< ::dynamixel_motor_config_t >
dynamixel_motor_config (const ::xercesc::DOMDocument& d,
                        ::xml_schema::flags f = 0,
                        const ::xml_schema::properties& p = ::xml_schema::properties ());

::std::auto_ptr< ::dynamixel_motor_config_t >
dynamixel_motor_config (::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d,
                        ::xml_schema::flags f = 0,
                        const ::xml_schema::properties& p = ::xml_schema::properties ());

#include <iosfwd>

#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMErrorHandler.hpp>
#include <xercesc/framework/XMLFormatter.hpp>

#include <xsd/cxx/xml/dom/auto-ptr.hxx>

void
operator<< (::xercesc::DOMElement&, const alarm_t&);

void
operator<< (::xercesc::DOMAttr&, const alarm_t&);

void
operator<< (::xml_schema::list_stream&,
            const alarm_t&);

void
operator<< (::xercesc::DOMElement&, const dynamixel_motor_config_t&);

// Serialize to std::ostream.
//

void
dynamixel_motor_config (::std::ostream& os,
                        const ::dynamixel_motor_config_t& x, 
                        const ::xml_schema::namespace_infomap& m = ::xml_schema::namespace_infomap (),
                        const ::std::string& e = "UTF-8",
                        ::xml_schema::flags f = 0);

void
dynamixel_motor_config (::std::ostream& os,
                        const ::dynamixel_motor_config_t& x, 
                        ::xml_schema::error_handler& eh,
                        const ::xml_schema::namespace_infomap& m = ::xml_schema::namespace_infomap (),
                        const ::std::string& e = "UTF-8",
                        ::xml_schema::flags f = 0);

void
dynamixel_motor_config (::std::ostream& os,
                        const ::dynamixel_motor_config_t& x, 
                        ::xercesc::DOMErrorHandler& eh,
                        const ::xml_schema::namespace_infomap& m = ::xml_schema::namespace_infomap (),
                        const ::std::string& e = "UTF-8",
                        ::xml_schema::flags f = 0);

// Serialize to xercesc::XMLFormatTarget.
//

void
dynamixel_motor_config (::xercesc::XMLFormatTarget& ft,
                        const ::dynamixel_motor_config_t& x, 
                        const ::xml_schema::namespace_infomap& m = ::xml_schema::namespace_infomap (),
                        const ::std::string& e = "UTF-8",
                        ::xml_schema::flags f = 0);

void
dynamixel_motor_config (::xercesc::XMLFormatTarget& ft,
                        const ::dynamixel_motor_config_t& x, 
                        ::xml_schema::error_handler& eh,
                        const ::xml_schema::namespace_infomap& m = ::xml_schema::namespace_infomap (),
                        const ::std::string& e = "UTF-8",
                        ::xml_schema::flags f = 0);

void
dynamixel_motor_config (::xercesc::XMLFormatTarget& ft,
                        const ::dynamixel_motor_config_t& x, 
                        ::xercesc::DOMErrorHandler& eh,
                        const ::xml_schema::namespace_infomap& m = ::xml_schema::namespace_infomap (),
                        const ::std::string& e = "UTF-8",
                        ::xml_schema::flags f = 0);

// Serialize to an existing xercesc::DOMDocument.
//

void
dynamixel_motor_config (::xercesc::DOMDocument& d,
                        const ::dynamixel_motor_config_t& x,
                        ::xml_schema::flags f = 0);

// Serialize to a new xercesc::DOMDocument.
//

::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument >
dynamixel_motor_config (const ::dynamixel_motor_config_t& x, 
                        const ::xml_schema::namespace_infomap& m = ::xml_schema::namespace_infomap (),
                        ::xml_schema::flags f = 0);

#include <xsd/cxx/post.hxx>

// Begin epilogue.
//
//
// End epilogue.

#endif // DYNAMIXEL_MOTOR_CFG_FILE_HXX
