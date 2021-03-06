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

// Begin prologue.
//
//
// End prologue.

#include <xsd/cxx/pre.hxx>

#include "dynamixel_motor_group_cfg_file.hxx"

// dyn_motor_config_t
// 

const dyn_motor_config_t::id_type& dyn_motor_config_t::
id () const
{
  return this->id_.get ();
}

dyn_motor_config_t::id_type& dyn_motor_config_t::
id ()
{
  return this->id_.get ();
}

void dyn_motor_config_t::
id (const id_type& x)
{
  this->id_.set (x);
}

const dyn_motor_config_t::config_file_type& dyn_motor_config_t::
config_file () const
{
  return this->config_file_.get ();
}

dyn_motor_config_t::config_file_type& dyn_motor_config_t::
config_file ()
{
  return this->config_file_.get ();
}

void dyn_motor_config_t::
config_file (const config_file_type& x)
{
  this->config_file_.set (x);
}

void dyn_motor_config_t::
config_file (::std::auto_ptr< config_file_type > x)
{
  this->config_file_.set (x);
}


// dyn_motor_group_config_t
// 

const dyn_motor_group_config_t::dynamixel_version_optional& dyn_motor_group_config_t::
dynamixel_version () const
{
  return this->dynamixel_version_;
}

dyn_motor_group_config_t::dynamixel_version_optional& dyn_motor_group_config_t::
dynamixel_version ()
{
  return this->dynamixel_version_;
}

void dyn_motor_group_config_t::
dynamixel_version (const dynamixel_version_type& x)
{
  this->dynamixel_version_.set (x);
}

void dyn_motor_group_config_t::
dynamixel_version (const dynamixel_version_optional& x)
{
  this->dynamixel_version_ = x;
}

const dyn_motor_group_config_t::dyn_motor_config_sequence& dyn_motor_group_config_t::
dyn_motor_config () const
{
  return this->dyn_motor_config_;
}

dyn_motor_group_config_t::dyn_motor_config_sequence& dyn_motor_group_config_t::
dyn_motor_config ()
{
  return this->dyn_motor_config_;
}

void dyn_motor_group_config_t::
dyn_motor_config (const dyn_motor_config_sequence& s)
{
  this->dyn_motor_config_ = s;
}


#include <xsd/cxx/xml/dom/parsing-source.hxx>

// dyn_motor_config_t
//

dyn_motor_config_t::
dyn_motor_config_t (const id_type& id,
                    const config_file_type& config_file)
: ::xml_schema::type (),
  id_ (id, this),
  config_file_ (config_file, this)
{
}

dyn_motor_config_t::
dyn_motor_config_t (const dyn_motor_config_t& x,
                    ::xml_schema::flags f,
                    ::xml_schema::container* c)
: ::xml_schema::type (x, f, c),
  id_ (x.id_, f, this),
  config_file_ (x.config_file_, f, this)
{
}

dyn_motor_config_t::
dyn_motor_config_t (const ::xercesc::DOMElement& e,
                    ::xml_schema::flags f,
                    ::xml_schema::container* c)
: ::xml_schema::type (e, f | ::xml_schema::flags::base, c),
  id_ (this),
  config_file_ (this)
{
  if ((f & ::xml_schema::flags::base) == 0)
  {
    ::xsd::cxx::xml::dom::parser< char > p (e, true, false, false);
    this->parse (p, f);
  }
}

void dyn_motor_config_t::
parse (::xsd::cxx::xml::dom::parser< char >& p,
       ::xml_schema::flags f)
{
  for (; p.more_content (); p.next_content (false))
  {
    const ::xercesc::DOMElement& i (p.cur_element ());
    const ::xsd::cxx::xml::qualified_name< char > n (
      ::xsd::cxx::xml::dom::name< char > (i));

    // id
    //
    if (n.name () == "id" && n.namespace_ ().empty ())
    {
      if (!id_.present ())
      {
        this->id_.set (id_traits::create (i, f, this));
        continue;
      }
    }

    // config_file
    //
    if (n.name () == "config_file" && n.namespace_ ().empty ())
    {
      ::std::auto_ptr< config_file_type > r (
        config_file_traits::create (i, f, this));

      if (!config_file_.present ())
      {
        this->config_file_.set (r);
        continue;
      }
    }

    break;
  }

  if (!id_.present ())
  {
    throw ::xsd::cxx::tree::expected_element< char > (
      "id",
      "");
  }

  if (!config_file_.present ())
  {
    throw ::xsd::cxx::tree::expected_element< char > (
      "config_file",
      "");
  }
}

dyn_motor_config_t* dyn_motor_config_t::
_clone (::xml_schema::flags f,
        ::xml_schema::container* c) const
{
  return new class dyn_motor_config_t (*this, f, c);
}

dyn_motor_config_t& dyn_motor_config_t::
operator= (const dyn_motor_config_t& x)
{
  if (this != &x)
  {
    static_cast< ::xml_schema::type& > (*this) = x;
    this->id_ = x.id_;
    this->config_file_ = x.config_file_;
  }

  return *this;
}

dyn_motor_config_t::
~dyn_motor_config_t ()
{
}

// dyn_motor_group_config_t
//

dyn_motor_group_config_t::
dyn_motor_group_config_t ()
: ::xml_schema::type (),
  dynamixel_version_ (this),
  dyn_motor_config_ (this)
{
}

dyn_motor_group_config_t::
dyn_motor_group_config_t (const dyn_motor_group_config_t& x,
                          ::xml_schema::flags f,
                          ::xml_schema::container* c)
: ::xml_schema::type (x, f, c),
  dynamixel_version_ (x.dynamixel_version_, f, this),
  dyn_motor_config_ (x.dyn_motor_config_, f, this)
{
}

dyn_motor_group_config_t::
dyn_motor_group_config_t (const ::xercesc::DOMElement& e,
                          ::xml_schema::flags f,
                          ::xml_schema::container* c)
: ::xml_schema::type (e, f | ::xml_schema::flags::base, c),
  dynamixel_version_ (this),
  dyn_motor_config_ (this)
{
  if ((f & ::xml_schema::flags::base) == 0)
  {
    ::xsd::cxx::xml::dom::parser< char > p (e, true, false, false);
    this->parse (p, f);
  }
}

void dyn_motor_group_config_t::
parse (::xsd::cxx::xml::dom::parser< char >& p,
       ::xml_schema::flags f)
{
  for (; p.more_content (); p.next_content (false))
  {
    const ::xercesc::DOMElement& i (p.cur_element ());
    const ::xsd::cxx::xml::qualified_name< char > n (
      ::xsd::cxx::xml::dom::name< char > (i));

    // dynamixel_version
    //
    if (n.name () == "dynamixel_version" && n.namespace_ ().empty ())
    {
      if (!this->dynamixel_version_)
      {
        this->dynamixel_version_.set (dynamixel_version_traits::create (i, f, this));
        continue;
      }
    }

    // dyn_motor_config
    //
    if (n.name () == "dyn_motor_config" && n.namespace_ ().empty ())
    {
      ::std::auto_ptr< dyn_motor_config_type > r (
        dyn_motor_config_traits::create (i, f, this));

      this->dyn_motor_config_.push_back (r);
      continue;
    }

    break;
  }
}

dyn_motor_group_config_t* dyn_motor_group_config_t::
_clone (::xml_schema::flags f,
        ::xml_schema::container* c) const
{
  return new class dyn_motor_group_config_t (*this, f, c);
}

dyn_motor_group_config_t& dyn_motor_group_config_t::
operator= (const dyn_motor_group_config_t& x)
{
  if (this != &x)
  {
    static_cast< ::xml_schema::type& > (*this) = x;
    this->dynamixel_version_ = x.dynamixel_version_;
    this->dyn_motor_config_ = x.dyn_motor_config_;
  }

  return *this;
}

dyn_motor_group_config_t::
~dyn_motor_group_config_t ()
{
}

#include <istream>
#include <xsd/cxx/xml/sax/std-input-source.hxx>
#include <xsd/cxx/tree/error-handler.hxx>

::std::auto_ptr< ::dyn_motor_group_config_t >
dyn_motor_group_config (const ::std::string& u,
                        ::xml_schema::flags f,
                        const ::xml_schema::properties& p)
{
  ::xsd::cxx::xml::auto_initializer i (
    (f & ::xml_schema::flags::dont_initialize) == 0,
    (f & ::xml_schema::flags::keep_dom) == 0);

  ::xsd::cxx::tree::error_handler< char > h;

  ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d (
    ::xsd::cxx::xml::dom::parse< char > (
      u, h, p, f));

  h.throw_if_failed< ::xsd::cxx::tree::parsing< char > > ();

  return ::std::auto_ptr< ::dyn_motor_group_config_t > (
    ::dyn_motor_group_config (
      d, f | ::xml_schema::flags::own_dom, p));
}

::std::auto_ptr< ::dyn_motor_group_config_t >
dyn_motor_group_config (const ::std::string& u,
                        ::xml_schema::error_handler& h,
                        ::xml_schema::flags f,
                        const ::xml_schema::properties& p)
{
  ::xsd::cxx::xml::auto_initializer i (
    (f & ::xml_schema::flags::dont_initialize) == 0,
    (f & ::xml_schema::flags::keep_dom) == 0);

  ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d (
    ::xsd::cxx::xml::dom::parse< char > (
      u, h, p, f));

  if (!d.get ())
    throw ::xsd::cxx::tree::parsing< char > ();

  return ::std::auto_ptr< ::dyn_motor_group_config_t > (
    ::dyn_motor_group_config (
      d, f | ::xml_schema::flags::own_dom, p));
}

::std::auto_ptr< ::dyn_motor_group_config_t >
dyn_motor_group_config (const ::std::string& u,
                        ::xercesc::DOMErrorHandler& h,
                        ::xml_schema::flags f,
                        const ::xml_schema::properties& p)
{
  ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d (
    ::xsd::cxx::xml::dom::parse< char > (
      u, h, p, f));

  if (!d.get ())
    throw ::xsd::cxx::tree::parsing< char > ();

  return ::std::auto_ptr< ::dyn_motor_group_config_t > (
    ::dyn_motor_group_config (
      d, f | ::xml_schema::flags::own_dom, p));
}

::std::auto_ptr< ::dyn_motor_group_config_t >
dyn_motor_group_config (::std::istream& is,
                        ::xml_schema::flags f,
                        const ::xml_schema::properties& p)
{
  ::xsd::cxx::xml::auto_initializer i (
    (f & ::xml_schema::flags::dont_initialize) == 0,
    (f & ::xml_schema::flags::keep_dom) == 0);

  ::xsd::cxx::xml::sax::std_input_source isrc (is);
  return ::dyn_motor_group_config (isrc, f, p);
}

::std::auto_ptr< ::dyn_motor_group_config_t >
dyn_motor_group_config (::std::istream& is,
                        ::xml_schema::error_handler& h,
                        ::xml_schema::flags f,
                        const ::xml_schema::properties& p)
{
  ::xsd::cxx::xml::auto_initializer i (
    (f & ::xml_schema::flags::dont_initialize) == 0,
    (f & ::xml_schema::flags::keep_dom) == 0);

  ::xsd::cxx::xml::sax::std_input_source isrc (is);
  return ::dyn_motor_group_config (isrc, h, f, p);
}

::std::auto_ptr< ::dyn_motor_group_config_t >
dyn_motor_group_config (::std::istream& is,
                        ::xercesc::DOMErrorHandler& h,
                        ::xml_schema::flags f,
                        const ::xml_schema::properties& p)
{
  ::xsd::cxx::xml::sax::std_input_source isrc (is);
  return ::dyn_motor_group_config (isrc, h, f, p);
}

::std::auto_ptr< ::dyn_motor_group_config_t >
dyn_motor_group_config (::std::istream& is,
                        const ::std::string& sid,
                        ::xml_schema::flags f,
                        const ::xml_schema::properties& p)
{
  ::xsd::cxx::xml::auto_initializer i (
    (f & ::xml_schema::flags::dont_initialize) == 0,
    (f & ::xml_schema::flags::keep_dom) == 0);

  ::xsd::cxx::xml::sax::std_input_source isrc (is, sid);
  return ::dyn_motor_group_config (isrc, f, p);
}

::std::auto_ptr< ::dyn_motor_group_config_t >
dyn_motor_group_config (::std::istream& is,
                        const ::std::string& sid,
                        ::xml_schema::error_handler& h,
                        ::xml_schema::flags f,
                        const ::xml_schema::properties& p)
{
  ::xsd::cxx::xml::auto_initializer i (
    (f & ::xml_schema::flags::dont_initialize) == 0,
    (f & ::xml_schema::flags::keep_dom) == 0);

  ::xsd::cxx::xml::sax::std_input_source isrc (is, sid);
  return ::dyn_motor_group_config (isrc, h, f, p);
}

::std::auto_ptr< ::dyn_motor_group_config_t >
dyn_motor_group_config (::std::istream& is,
                        const ::std::string& sid,
                        ::xercesc::DOMErrorHandler& h,
                        ::xml_schema::flags f,
                        const ::xml_schema::properties& p)
{
  ::xsd::cxx::xml::sax::std_input_source isrc (is, sid);
  return ::dyn_motor_group_config (isrc, h, f, p);
}

::std::auto_ptr< ::dyn_motor_group_config_t >
dyn_motor_group_config (::xercesc::InputSource& i,
                        ::xml_schema::flags f,
                        const ::xml_schema::properties& p)
{
  ::xsd::cxx::tree::error_handler< char > h;

  ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d (
    ::xsd::cxx::xml::dom::parse< char > (
      i, h, p, f));

  h.throw_if_failed< ::xsd::cxx::tree::parsing< char > > ();

  return ::std::auto_ptr< ::dyn_motor_group_config_t > (
    ::dyn_motor_group_config (
      d, f | ::xml_schema::flags::own_dom, p));
}

::std::auto_ptr< ::dyn_motor_group_config_t >
dyn_motor_group_config (::xercesc::InputSource& i,
                        ::xml_schema::error_handler& h,
                        ::xml_schema::flags f,
                        const ::xml_schema::properties& p)
{
  ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d (
    ::xsd::cxx::xml::dom::parse< char > (
      i, h, p, f));

  if (!d.get ())
    throw ::xsd::cxx::tree::parsing< char > ();

  return ::std::auto_ptr< ::dyn_motor_group_config_t > (
    ::dyn_motor_group_config (
      d, f | ::xml_schema::flags::own_dom, p));
}

::std::auto_ptr< ::dyn_motor_group_config_t >
dyn_motor_group_config (::xercesc::InputSource& i,
                        ::xercesc::DOMErrorHandler& h,
                        ::xml_schema::flags f,
                        const ::xml_schema::properties& p)
{
  ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d (
    ::xsd::cxx::xml::dom::parse< char > (
      i, h, p, f));

  if (!d.get ())
    throw ::xsd::cxx::tree::parsing< char > ();

  return ::std::auto_ptr< ::dyn_motor_group_config_t > (
    ::dyn_motor_group_config (
      d, f | ::xml_schema::flags::own_dom, p));
}

::std::auto_ptr< ::dyn_motor_group_config_t >
dyn_motor_group_config (const ::xercesc::DOMDocument& doc,
                        ::xml_schema::flags f,
                        const ::xml_schema::properties& p)
{
  if (f & ::xml_schema::flags::keep_dom)
  {
    ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d (
      static_cast< ::xercesc::DOMDocument* > (doc.cloneNode (true)));

    return ::std::auto_ptr< ::dyn_motor_group_config_t > (
      ::dyn_motor_group_config (
        d, f | ::xml_schema::flags::own_dom, p));
  }

  const ::xercesc::DOMElement& e (*doc.getDocumentElement ());
  const ::xsd::cxx::xml::qualified_name< char > n (
    ::xsd::cxx::xml::dom::name< char > (e));

  if (n.name () == "dyn_motor_group_config" &&
      n.namespace_ () == "")
  {
    ::std::auto_ptr< ::dyn_motor_group_config_t > r (
      ::xsd::cxx::tree::traits< ::dyn_motor_group_config_t, char >::create (
        e, f, 0));
    return r;
  }

  throw ::xsd::cxx::tree::unexpected_element < char > (
    n.name (),
    n.namespace_ (),
    "dyn_motor_group_config",
    "");
}

::std::auto_ptr< ::dyn_motor_group_config_t >
dyn_motor_group_config (::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d,
                        ::xml_schema::flags f,
                        const ::xml_schema::properties&)
{
  ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > c (
    ((f & ::xml_schema::flags::keep_dom) &&
     !(f & ::xml_schema::flags::own_dom))
    ? static_cast< ::xercesc::DOMDocument* > (d->cloneNode (true))
    : 0);

  ::xercesc::DOMDocument& doc (c.get () ? *c : *d);
  const ::xercesc::DOMElement& e (*doc.getDocumentElement ());

  const ::xsd::cxx::xml::qualified_name< char > n (
    ::xsd::cxx::xml::dom::name< char > (e));

  if (f & ::xml_schema::flags::keep_dom)
    doc.setUserData (::xml_schema::dom::tree_node_key,
                     (c.get () ? &c : &d),
                     0);

  if (n.name () == "dyn_motor_group_config" &&
      n.namespace_ () == "")
  {
    ::std::auto_ptr< ::dyn_motor_group_config_t > r (
      ::xsd::cxx::tree::traits< ::dyn_motor_group_config_t, char >::create (
        e, f, 0));
    return r;
  }

  throw ::xsd::cxx::tree::unexpected_element < char > (
    n.name (),
    n.namespace_ (),
    "dyn_motor_group_config",
    "");
}

#include <ostream>
#include <xsd/cxx/tree/error-handler.hxx>
#include <xsd/cxx/xml/dom/serialization-source.hxx>

void
operator<< (::xercesc::DOMElement& e, const dyn_motor_config_t& i)
{
  e << static_cast< const ::xml_schema::type& > (i);

  // id
  //
  {
    ::xercesc::DOMElement& s (
      ::xsd::cxx::xml::dom::create_element (
        "id",
        e));

    s << i.id ();
  }

  // config_file
  //
  {
    ::xercesc::DOMElement& s (
      ::xsd::cxx::xml::dom::create_element (
        "config_file",
        e));

    s << i.config_file ();
  }
}

void
operator<< (::xercesc::DOMElement& e, const dyn_motor_group_config_t& i)
{
  e << static_cast< const ::xml_schema::type& > (i);

  // dynamixel_version
  //
  if (i.dynamixel_version ())
  {
    ::xercesc::DOMElement& s (
      ::xsd::cxx::xml::dom::create_element (
        "dynamixel_version",
        e));

    s << *i.dynamixel_version ();
  }

  // dyn_motor_config
  //
  for (dyn_motor_group_config_t::dyn_motor_config_const_iterator
       b (i.dyn_motor_config ().begin ()), n (i.dyn_motor_config ().end ());
       b != n; ++b)
  {
    ::xercesc::DOMElement& s (
      ::xsd::cxx::xml::dom::create_element (
        "dyn_motor_config",
        e));

    s << *b;
  }
}

void
dyn_motor_group_config (::std::ostream& o,
                        const ::dyn_motor_group_config_t& s,
                        const ::xml_schema::namespace_infomap& m,
                        const ::std::string& e,
                        ::xml_schema::flags f)
{
  ::xsd::cxx::xml::auto_initializer i (
    (f & ::xml_schema::flags::dont_initialize) == 0);

  ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d (
    ::dyn_motor_group_config (s, m, f));

  ::xsd::cxx::tree::error_handler< char > h;

  ::xsd::cxx::xml::dom::ostream_format_target t (o);
  if (!::xsd::cxx::xml::dom::serialize (t, *d, e, h, f))
  {
    h.throw_if_failed< ::xsd::cxx::tree::serialization< char > > ();
  }
}

void
dyn_motor_group_config (::std::ostream& o,
                        const ::dyn_motor_group_config_t& s,
                        ::xml_schema::error_handler& h,
                        const ::xml_schema::namespace_infomap& m,
                        const ::std::string& e,
                        ::xml_schema::flags f)
{
  ::xsd::cxx::xml::auto_initializer i (
    (f & ::xml_schema::flags::dont_initialize) == 0);

  ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d (
    ::dyn_motor_group_config (s, m, f));
  ::xsd::cxx::xml::dom::ostream_format_target t (o);
  if (!::xsd::cxx::xml::dom::serialize (t, *d, e, h, f))
  {
    throw ::xsd::cxx::tree::serialization< char > ();
  }
}

void
dyn_motor_group_config (::std::ostream& o,
                        const ::dyn_motor_group_config_t& s,
                        ::xercesc::DOMErrorHandler& h,
                        const ::xml_schema::namespace_infomap& m,
                        const ::std::string& e,
                        ::xml_schema::flags f)
{
  ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d (
    ::dyn_motor_group_config (s, m, f));
  ::xsd::cxx::xml::dom::ostream_format_target t (o);
  if (!::xsd::cxx::xml::dom::serialize (t, *d, e, h, f))
  {
    throw ::xsd::cxx::tree::serialization< char > ();
  }
}

void
dyn_motor_group_config (::xercesc::XMLFormatTarget& t,
                        const ::dyn_motor_group_config_t& s,
                        const ::xml_schema::namespace_infomap& m,
                        const ::std::string& e,
                        ::xml_schema::flags f)
{
  ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d (
    ::dyn_motor_group_config (s, m, f));

  ::xsd::cxx::tree::error_handler< char > h;

  if (!::xsd::cxx::xml::dom::serialize (t, *d, e, h, f))
  {
    h.throw_if_failed< ::xsd::cxx::tree::serialization< char > > ();
  }
}

void
dyn_motor_group_config (::xercesc::XMLFormatTarget& t,
                        const ::dyn_motor_group_config_t& s,
                        ::xml_schema::error_handler& h,
                        const ::xml_schema::namespace_infomap& m,
                        const ::std::string& e,
                        ::xml_schema::flags f)
{
  ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d (
    ::dyn_motor_group_config (s, m, f));
  if (!::xsd::cxx::xml::dom::serialize (t, *d, e, h, f))
  {
    throw ::xsd::cxx::tree::serialization< char > ();
  }
}

void
dyn_motor_group_config (::xercesc::XMLFormatTarget& t,
                        const ::dyn_motor_group_config_t& s,
                        ::xercesc::DOMErrorHandler& h,
                        const ::xml_schema::namespace_infomap& m,
                        const ::std::string& e,
                        ::xml_schema::flags f)
{
  ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d (
    ::dyn_motor_group_config (s, m, f));
  if (!::xsd::cxx::xml::dom::serialize (t, *d, e, h, f))
  {
    throw ::xsd::cxx::tree::serialization< char > ();
  }
}

void
dyn_motor_group_config (::xercesc::DOMDocument& d,
                        const ::dyn_motor_group_config_t& s,
                        ::xml_schema::flags)
{
  ::xercesc::DOMElement& e (*d.getDocumentElement ());
  const ::xsd::cxx::xml::qualified_name< char > n (
    ::xsd::cxx::xml::dom::name< char > (e));

  if (n.name () == "dyn_motor_group_config" &&
      n.namespace_ () == "")
  {
    e << s;
  }
  else
  {
    throw ::xsd::cxx::tree::unexpected_element < char > (
      n.name (),
      n.namespace_ (),
      "dyn_motor_group_config",
      "");
  }
}

::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument >
dyn_motor_group_config (const ::dyn_motor_group_config_t& s,
                        const ::xml_schema::namespace_infomap& m,
                        ::xml_schema::flags f)
{
  ::xml_schema::dom::auto_ptr< ::xercesc::DOMDocument > d (
    ::xsd::cxx::xml::dom::serialize< char > (
      "dyn_motor_group_config",
      "",
      m, f));

  ::dyn_motor_group_config (*d, s, f);
  return d;
}

#include <xsd/cxx/post.hxx>

// Begin epilogue.
//
//
// End epilogue.

