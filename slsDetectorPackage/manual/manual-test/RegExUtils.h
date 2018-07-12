/********************************************************************************/
/*! 
 *  \file   Receivers.h
 *  \brief  regex class interface 
*/
/********************************************************************************/

#ifndef REGEX_H
#define REGEX_H

// SYSTEM
#include <string>
#include <vector>
#include <map>
#include <sys/types.h>
#include <regex.h>

namespace Detector_ns
{
    /***********************************************************************
     * \class RegExException
     ***********************************************************************/
    struct RegExException : public std::exception
    {
       std::string s;

       RegExException(std::string ss) : s(ss) {}

       ~RegExException() throw () {} // Updated

       const char* what() const throw() { return s.c_str(); }
    };

    /***********************************************************************
     * \class SimpleRegEx
     ***********************************************************************/
    class SimpleRegEx
    {
     public:
        //==================================================================
	    typedef struct SingleMatch 
        {
		    typedef std::string::const_iterator StrIt;

		    StrIt start;
		    StrIt end  ;

		    SingleMatch();
		    SingleMatch(StrIt it, const regmatch_t& rm);

		    bool found() const;

		    operator std::string() const;
	    } SingleMatchType;

        //==================================================================
	    typedef std::vector<SingleMatchType> FullMatchType;
	    typedef std::vector<FullMatchType>   MatchListType;

        //==================================================================
	    SimpleRegEx();
	    SimpleRegEx(const std::string& regex_str);
	    SimpleRegEx(const SimpleRegEx& regex);
	    ~SimpleRegEx();

        //==================================================================
	    SimpleRegEx& operator  =(const std::string& regex_str);
	    SimpleRegEx& operator +=(const std::string& regex_str);
	    SimpleRegEx& operator  =(const SimpleRegEx& regex);
	    SimpleRegEx& operator +=(const SimpleRegEx& regex);

        //==================================================================
	    const std::string& getRegExStr() const;

	    bool single_search(const std::string& str, FullMatchType& match, int nb_groups = 0, int match_idx = 0) const;

        void multi_search(const std::string& str, MatchListType& match_list, int nb_groups = 0, int max_nb_match = 0) const;

        bool match(const std::string& str, FullMatchType& match, int nb_groups = 0) const;

	    int get_nb_groups() const;

     private:
	    void set(const std::string& regex_str);
	    void free();

	    static int find_nb_groups(const std::string& regex_str);

	    std::string str_error(int ret) const;

	    std::string m_str;
	    regex_t     m_regex;
	    int         m_nb_groups;
    };

    SimpleRegEx operator +(const SimpleRegEx& re1, const SimpleRegEx& re2);

    /***********************************************************************
     * \class SimpleRegEx
     ***********************************************************************/
    class RegEx 
    {
     public:
	    typedef SimpleRegEx::SingleMatchType           SingleMatchType;
	    typedef SimpleRegEx::FullMatchType             FullMatchType;
	    typedef SimpleRegEx::MatchListType             MatchListType;

	    typedef std::map<std::string, SingleMatchType> FullNameMatchType;
	    typedef std::vector<FullNameMatchType>         NameMatchListType;

        //==================================================================
	    RegEx();
	    RegEx(const std::string& regex_str);
	    RegEx(const RegEx& regex);
	    ~RegEx();

        //==================================================================
	    RegEx& operator  =(const std::string& regex_str);
	    RegEx& operator +=(const std::string& regex_str);
	    RegEx& operator  =(const RegEx& regex);
	    RegEx& operator +=(const RegEx& regex);

        //==================================================================
	    const std::string& getRegExStr() const;

	    bool single_search(const std::string& str, FullMatchType& match, int nb_groups = 0, int match_idx = 0) const;

        void multi_search(const std::string& str, MatchListType& match_list, int nb_groups = 0, int max_nb_match = 0) const;

        bool match(const std::string& str, FullMatchType& match, int nb_groups = 0) const;

	    bool single_search_name(const std::string& str, FullNameMatchType& name_match, int match_idx = 0) const;

        void multi_search_name(const std::string& str, NameMatchListType& name_match_list, int max_nb_match = 0) const;

        bool match_name(const std::string& str, FullNameMatchType& name_match) const;

	    int get_nb_groups() const;
	    int get_nb_name_groups() const;

     private:
	    typedef std::map<std::string, int> NameMapType;

	    void set(const std::string& regex_str);

        void free();

        void convert_name_match(const FullMatchType& match, FullNameMatchType& name_match) const;

	    std::string m_str;
	    SimpleRegEx m_regex;
	    NameMapType m_name_map;
    };

    RegEx operator +(const RegEx& re1, const RegEx& re2);

} // namespace Detector_ns

#endif // REGEX_H
