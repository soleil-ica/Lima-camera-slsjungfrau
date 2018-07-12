/*************************************************************************************/
/*! 
 *  \file   RegExUtils.h
 *  \brief  regex class implementation
*/
/*************************************************************************************/

// LOCAL
#include "RegExUtils.h"

using namespace std;
using namespace Detector_ns;

/************************************************************************
 * \brief constructor
 ************************************************************************/
SimpleRegEx::SingleMatch::SingleMatch()
{
	start = end;
}

/************************************************************************
 * \brief constructor
 ************************************************************************/
SimpleRegEx::SingleMatch::SingleMatch(StrIt it, const regmatch_t& rm)
{
	if ((rm.rm_so != -1) && (rm.rm_eo != -1)) {
		start = it + rm.rm_so;
		end   = it + rm.rm_eo;
	} else
		start = end = it;
}

/************************************************************************
 * \brief found
 ************************************************************************/
bool SimpleRegEx::SingleMatch::found() const
{ 
	return start != end; 
}

/************************************************************************
 * \brief string
 ************************************************************************/
SimpleRegEx::SingleMatch::operator string() const
{ 
	return string(start, end); 
}

/************************************************************************
 * \brief constructor
 ************************************************************************/
SimpleRegEx::SimpleRegEx()
{
	set("");
}

/************************************************************************
 * \brief constructor
 ************************************************************************/
SimpleRegEx::SimpleRegEx(const string& regex_str)
{
	set(regex_str);
}

/************************************************************************
 * \brief constructor
 ************************************************************************/
SimpleRegEx::SimpleRegEx(const SimpleRegEx& regex)
{
	set(regex.m_str);
}

/************************************************************************
 * \brief destructor
 ************************************************************************/
SimpleRegEx::~SimpleRegEx()
{
	free();
}

/************************************************************************
 * \brief operator =
 ************************************************************************/
SimpleRegEx& SimpleRegEx::operator =(const string& regex_str)
{
	set(regex_str);
	return *this;
}

/************************************************************************
 * \brief operator +=
 ************************************************************************/
SimpleRegEx& SimpleRegEx::operator +=(const string& regex_str)
{
	set(m_str + regex_str);
	return *this;
}

/************************************************************************
 * \brief operator =
 ************************************************************************/
SimpleRegEx& SimpleRegEx::operator =(const SimpleRegEx& regex)
{
	set(regex.m_str);
	return *this;
}

/************************************************************************
 * \brief operator +=
 ************************************************************************/
SimpleRegEx& SimpleRegEx::operator +=(const SimpleRegEx& regex)
{
	set(m_str + regex.m_str);
	return *this;
}

/************************************************************************
 * \brief set
 ************************************************************************/
void SimpleRegEx::set(const string& regex_str)
{
	if (regex_str == m_str)
		return;

	free();

	if (!regex_str.empty())
    {
		int aux_ret = regcomp(&m_regex, regex_str.c_str(), REG_EXTENDED);
		
        if (aux_ret != 0)
			throw RegExException(str_error(aux_ret));
	}

	m_str = regex_str;
	m_nb_groups = find_nb_groups(regex_str);
}

/************************************************************************
 * \brief find_nb_groups
 ************************************************************************/
int SimpleRegEx::find_nb_groups(const string& regex_str)
{
	int nb_groups = 0;
	string::const_iterator it, end = regex_str.end();
	bool in_escape = false;

    for (it = regex_str.begin(); it != end; ++it) 
    {
		switch (*it) 
        {
		case '\\':
			in_escape = !in_escape;
			break;
		case '(':
			if (!in_escape)
				nb_groups++;
		default:
			in_escape = false;
		}
	}

	return nb_groups;
}

/************************************************************************
 * \brief free
 ************************************************************************/
void SimpleRegEx::free()
{
	if (m_str.empty())
		return;

	regfree(&m_regex);
	m_str.clear();
}

/************************************************************************
 * \brief getRegExStr
 ************************************************************************/
const string& SimpleRegEx::getRegExStr() const
{
	return m_str;
}

/************************************************************************
 * \brief get_nb_groups
 ************************************************************************/
int SimpleRegEx::get_nb_groups() const
{
	return m_nb_groups;
}

/************************************************************************
 * \brief get_nb_groups
 ************************************************************************/
bool SimpleRegEx::single_search(const string  & str      ,
                                FullMatchType & match    ,
                                int             nb_groups,
                                int             match_idx) const
{
	if (match_idx < 0)
		throw RegExException("Invalid match index");

	MatchListType match_list;
	multi_search(str, match_list, nb_groups, match_idx + 1);

    if (int(match_list.size()) <= match_idx)
		return false;

	match = match_list[match_idx];
	return true;
}

/************************************************************************
 * \brief multi_search
 ************************************************************************/
void SimpleRegEx::multi_search(const string  & str         ,
                               MatchListType & match_list  ,
                               int             nb_groups   ,
                               int             max_nb_match) const
{
	if (m_str.empty())
		throw RegExException("Regular expression not set");

	match_list.clear();

	typedef string::const_iterator StrIt;
	StrIt sbeg = str.begin();
	StrIt send = str.end();

	nb_groups = (nb_groups > 0) ? nb_groups : (m_nb_groups + 1);

	regmatch_t reg_match[nb_groups];
	regmatch_t *mend = reg_match + nb_groups;
	StrIt it = sbeg; 

    for (int i = 0; it != send; i++) 
    {
		if ((max_nb_match > 0) && (i == max_nb_match))
			break;

		string aux(it, send);
		int flags = (it != sbeg) ? REG_NOTBOL : 0;
		int ret = regexec(&m_regex, aux.c_str(), nb_groups, reg_match, 
				  flags);
		if (ret == REG_NOMATCH)
			break;

        if (ret != 0)
			throw RegExException(str_error(ret));

		FullMatchType full_match;

        for (regmatch_t *m = reg_match; m != mend; ++m) 
        {
			SingleMatchType match(it, *m);
			full_match.push_back(match);
		}		

        match_list.push_back(full_match);

		it += reg_match[0].rm_eo;
	}
}

/************************************************************************
 * \brief match
 ************************************************************************/
bool SimpleRegEx::match(const string  & str      ,
                        FullMatchType & match    ,
                        int             nb_groups) const
{
	if (!single_search(str, match, nb_groups))
		return false;
	
	return (match[0].start == str.begin());
}

/************************************************************************
 * \brief str_error
 ************************************************************************/
std::string SimpleRegEx::str_error(int ret) const
{
	size_t len = regerror(ret, &m_regex, NULL, 0);
	string regerr(len, '\0');
	char *data = (char *) regerr.data();
	regerror(ret, &m_regex, data, regerr.size());
	return regerr;
}

/************************************************************************
 * \brief operator +
 ************************************************************************/
SimpleRegEx Detector_ns::operator +(const SimpleRegEx& re1, const SimpleRegEx& re2)
{
	SimpleRegEx re = re1;
	return re += re2;
}

/************************************************************************
 * \brief constructor
 ************************************************************************/
RegEx::RegEx()
{
	set("");
}

/************************************************************************
 * \brief constructor
 ************************************************************************/
RegEx::RegEx(const string& regex_str)
{
	set(regex_str);
}

/************************************************************************
 * \brief constructor
 ************************************************************************/
RegEx::RegEx(const RegEx& regex)
{
	set(regex.m_str);
}

/************************************************************************
 * \brief destructor
 ************************************************************************/
RegEx::~RegEx()
{
	free();
}

/************************************************************************
 * \brief operator =
 ************************************************************************/
RegEx& RegEx::operator =(const string& regex_str)
{
	set(regex_str);
	return *this;
}

/************************************************************************
 * \brief operator +=
 ************************************************************************/
RegEx& RegEx::operator +=(const string& regex_str)
{
	set(m_str + regex_str);
	return *this;
}

/************************************************************************
 * \brief operator =
 ************************************************************************/
RegEx& RegEx::operator =(const RegEx& regex)
{
	set(regex.m_str);
	return *this;
}

/************************************************************************
 * \brief operator +=
 ************************************************************************/
RegEx& RegEx::operator +=(const RegEx& regex)
{
	set(m_str + regex.m_str);
	return *this;
}

/************************************************************************
 * \brief free
 ************************************************************************/
void RegEx::free()
{
	m_name_map.clear();
	m_regex = "";
	m_str.clear();
}

/************************************************************************
 * \brief set
 ************************************************************************/
void RegEx::set(const string& regex_str)
{
	if (regex_str == m_str)
		return;

	free();

	string re = "([^(])?(\\()(\\?P<([A-Za-z][A-Za-z0-9_]*)>)?[^\\(]*";
	SimpleRegEx grp_start_re(re);

	typedef SimpleRegEx::SingleMatchType SingleMatchType;
	typedef SimpleRegEx::FullMatchType   FullMatchType;
	typedef SimpleRegEx::MatchListType   MatchListType;

	MatchListType grp_list;
	grp_start_re.multi_search(regex_str, grp_list);

	string::const_iterator sit = regex_str.begin();
	int grp_nb = 0;
	string simple_regex_str;

	MatchListType::const_iterator mit, mend = grp_list.end();
	for (mit = grp_list.begin(); mit != mend; ++mit) 
    {
		const FullMatchType& fm = *mit;
		const SingleMatchType& grp_start   = fm[0];
		const SingleMatchType& pre_grp_chr = fm[1];
		const SingleMatchType& grp_open    = fm[2];
		const SingleMatchType& grp_ext     = fm[3];
		const SingleMatchType& grp_name    = fm[4];

		simple_regex_str += string(sit, grp_open.end);
		sit = grp_open.end;

		bool is_grp = (!pre_grp_chr.found() || 
			       (*pre_grp_chr.start != '\\'));
		if (is_grp)
			grp_nb++;

        if (is_grp && grp_ext.found()) 
        {
			m_name_map[grp_name] = grp_nb;
			sit = grp_ext.end;
		}

        simple_regex_str += string(sit, grp_start.end);
		sit = grp_start.end;
	}
	simple_regex_str += string(sit, regex_str.end());
	
	m_str = regex_str;
	m_regex = simple_regex_str;

	if (m_regex.get_nb_groups() != grp_nb)
		throw RegExException("RegEx nb of groups mismatch");
}

/************************************************************************
 * \brief getRegExStr
 ************************************************************************/
const string& RegEx::getRegExStr() const
{
	return m_str;
}

/************************************************************************
 * \brief get_nb_groups
 ************************************************************************/
int RegEx::get_nb_groups() const
{
	return m_regex.get_nb_groups();
}

/************************************************************************
 * \brief get_nb_name_groups
 ************************************************************************/
int RegEx::get_nb_name_groups() const
{
	return m_name_map.size();
}

/************************************************************************
 * \brief single_search
 ************************************************************************/
bool RegEx::single_search(const string  & str      ,
                          FullMatchType & match    ,
			              int             nb_groups,
                          int             match_idx) const
{
	return m_regex.single_search(str, match, nb_groups, match_idx);
}

/************************************************************************
 * \brief single_search_name
 ************************************************************************/
bool RegEx::single_search_name(const string      & str       , 
			                   FullNameMatchType & name_match, 
			                   int                 match_idx ) const
{
	name_match.clear();

	FullMatchType full_match;

    if (!single_search(str, full_match, 0, match_idx))
		return false;

	convert_name_match(full_match, name_match);
	return true;
}

/************************************************************************
 * \brief convert_name_match
 ************************************************************************/
void RegEx::convert_name_match(const FullMatchType & full_match, 
			                   FullNameMatchType   & name_match) const
{
	NameMapType::const_iterator it, end = m_name_map.end();

    for (it = m_name_map.begin(); it != end; ++it) 
    {
		const string& name  = it->first;
		int group_nb = it->second;
		const SingleMatchType& match = full_match[group_nb];
		name_match[name] = match;
	}
}

/************************************************************************
 * \brief multi_search
 ************************************************************************/
void RegEx::multi_search(const string  & str         ,
                         MatchListType & match_list  ,
                         int             nb_groups   ,
                         int             max_nb_match) const
{
	m_regex.multi_search(str, match_list, nb_groups, max_nb_match);
}

/************************************************************************
 * \brief multi_search_name
 ************************************************************************/
void RegEx::multi_search_name(const string      & str            , 
			                  NameMatchListType & name_match_list,
                              int                 max_nb_match   ) const
{
	name_match_list.clear();

	MatchListType match_list;
	multi_search(str, match_list, 0, max_nb_match);

	MatchListType::const_iterator it, end = match_list.end();

    for (it = match_list.begin(); it != end; ++it) 
    {
		FullNameMatchType name_match;
		convert_name_match(*it, name_match);
		name_match_list.push_back(name_match);
	}
}

/************************************************************************
 * \brief match
 ************************************************************************/
bool RegEx::match(const string  & str      ,
                  FullMatchType & match    ,
                  int             nb_groups) const
{
	return m_regex.match(str, match, nb_groups);
}

/************************************************************************
 * \brief match_name
 ************************************************************************/
bool RegEx::match_name(const string      & str       , 
		               FullNameMatchType & name_match) const
{
	name_match.clear();

	FullMatchType full_match;

	if (!match(str, full_match, 0))
		return false;

	convert_name_match(full_match, name_match);
	return true;
}

/************************************************************************
 * \brief operator +
 ************************************************************************/
RegEx Detector_ns::operator +(const RegEx& re1, const RegEx& re2)
{
	RegEx re = re1;
	return re += re2;
}
