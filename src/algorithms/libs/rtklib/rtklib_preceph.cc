/*!
 * \file rtklib_preceph.cc
 * \brief precise ephemeris and clock functions
 * \authors <ul>
 *          <li> 2007-2013, T. Takasu
 *          <li> 2017, Javier Arribas
 *          <li> 2017, Carles Fernandez
 *          </ul>
 *
 * This is a derived work from RTKLIB http://www.rtklib.com/
 * The original source code at https://github.com/tomojitakasu/RTKLIB is
 * released under the BSD 2-clause license with an additional exclusive clause
 * that does not apply here. This additional clause is reproduced below:
 *
 * " The software package includes some companion executive binaries or shared
 * libraries necessary to execute APs on Windows. These licenses succeed to the
 * original ones of these software. "
 *
 * Neither the executive binaries nor the shared libraries are required by, used
 * or included in GNSS-SDR.
 *
 * -------------------------------------------------------------------------
 * Copyright (C) 2007-2013, T. Takasu
 * Copyright (C) 2017, Javier Arribas
 * Copyright (C) 2017, Carles Fernandez
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * References :
 *     [1] S.Hilla, The Extended Standard Product 3 Orbit Format (SP3-c),
 *         12 February, 2007
 *     [2] J.Ray, W.Gurtner, RINEX Extensions to Handle Clock Information,
 *         27 August, 1998
 *     [3] D.D.McCarthy, IERS Technical Note 21, IERS Conventions 1996, July 1996
 *     [4] D.A.Vallado, Fundamentals of Astrodynamics and Applications 2nd ed,
 *         Space Technology Library, 2004
 *
 *----------------------------------------------------------------------------*/

#include "rtklib_preceph.h"
#include "rtklib_rtkcmn.h"

/* satellite code to satellite system ----------------------------------------*/
int code2sys(char code)
{
    if (code == 'G' || code == ' ') return SYS_GPS;
    if (code == 'R') return SYS_GLO;
    if (code == 'E') return SYS_GAL; /* extension to sp3-c */
    if (code == 'J') return SYS_QZS; /* extension to sp3-c */
    if (code == 'C') return SYS_BDS; /* extension to sp3-c */
    if (code == 'L') return SYS_LEO; /* extension to sp3-c */
    return SYS_NONE;
}


/* read sp3 header -----------------------------------------------------------*/
int readsp3h(FILE *fp, gtime_t *time, char *type, int *sats,
        double *bfact, char *tsys)
{
    int i, j, k = 0, ns = 0, sys, prn;
    char buff[1024];

    trace(3, "readsp3h:\n");

    for (i = 0;i<22;i++)
        {
            if (!fgets(buff, sizeof(buff), fp)) break;

            if (i == 0)
                {
                    *type = buff[2];
                    if (str2time(buff, 3, 28, time)) return 0;
                }
            else if (2<=i && i<=6)
                {
                    if (i == 2)
                        {
                            ns = (int)str2num(buff, 4, 2);
                        }
                    for (j = 0;j<17 && k<ns;j++)
                        {
                            sys = code2sys(buff[9+3*j]);
                            prn = (int)str2num(buff, 10+3*j, 2);
                            if (k<MAXSAT) sats[k++] = satno(sys, prn);
                        }
                }
            else if (i == 12)
                {
                    strncpy(tsys, buff+9, 3); tsys[3] = '\0';
                }
            else if (i == 14)
                {
                    bfact[0] = str2num(buff,  3, 10);
                    bfact[1] = str2num(buff, 14, 12);
                }
        }
    return ns;
}


/* add precise ephemeris -----------------------------------------------------*/
int addpeph(nav_t *nav, peph_t *peph)
{
    peph_t *nav_peph;

    if (nav->ne >= nav->nemax)
        {
            nav->nemax += 256;
            if (!(nav_peph = (peph_t *)realloc(nav->peph, sizeof(peph_t)*nav->nemax)))
                {
                    trace(1, "readsp3b malloc error n=%d\n", nav->nemax);
                    free(nav->peph);
                    nav->peph = NULL;
                    nav->ne = nav->nemax = 0;
                    return 0;
                }
            nav->peph = nav_peph;
        }
    nav->peph[nav->ne++] = *peph;
    return 1;
}


/* read sp3 body -------------------------------------------------------------*/
void readsp3b(FILE *fp, char type, int *sats __attribute__((unused)), int ns, double *bfact,
        char *tsys, int index, int opt, nav_t *nav)
{
    peph_t peph;
    gtime_t time;
    double val, std, base;
    int i, j, sat, sys, prn, n = ns*(type == 'P' ? 1 : 2), pred_o, pred_c, v;
    char buff[1024];

    trace(3, "readsp3b: type=%c ns=%d index=%d opt=%d\n", type, ns, index, opt);

    while (fgets(buff, sizeof(buff), fp))
        {
            if (!strncmp(buff, "EOF", 3)) break;

            if (buff[0] != '*' || str2time(buff, 3, 28, &time))
                {
                    trace(2, "sp3 invalid epoch %31.31s\n", buff);
                    continue;
                }
            if (!strcmp(tsys, "UTC")) time = utc2gpst(time); /* utc->gpst */
            peph.time  = time;
            peph.index = index;

            for (i = 0; i < MAXSAT; i++)
                {
                    for (j = 0; j < 4; j++)
                        {
                            peph.pos[i][j] = 0.0;
                            peph.std[i][j] = 0.0f;
                            peph.vel[i][j] = 0.0;
                            peph.vst[i][j] = 0.0f;
                        }
                    for (j = 0; j < 3; j++)
                        {
                            peph.cov[i][j] = 0.0f;
                            peph.vco[i][j] = 0.0f;
                        }
                }
            for (i = pred_o = pred_c = v = 0;i<n && fgets(buff, sizeof(buff), fp);i++)
                {
                    if (strlen(buff) < 4 || (buff[0] != 'P' && buff[0] != 'V')) continue;

                    sys = buff[1] == ' '?SYS_GPS:code2sys(buff[1]);
                    prn = (int)str2num(buff, 2, 2);
                    if      (sys == SYS_SBS) prn += 100;
                    else if (sys == SYS_QZS) prn += 192; /* extension to sp3-c */

                    if (!(sat = satno(sys, prn))) continue;

                    if (buff[0] == 'P')
                        {
                            pred_c = strlen(buff) >= 76 && buff[75] == 'P';
                            pred_o = strlen(buff) >= 80 && buff[79] == 'P';
                        }
                    for (j = 0;j<4;j++)
                        {
                            /* read option for predicted value */
                            if (j< 3 && (opt&1) &&  pred_o) continue;
                            if (j< 3 && (opt&2) && !pred_o) continue;
                            if (j == 3 && (opt&1) &&  pred_c) continue;
                            if (j == 3 && (opt&2) && !pred_c) continue;

                            val = str2num(buff,  4 + j * 14, 14);
                            std = str2num(buff, 61 + j * 3, j < 3 ? 2 : 3);

                            if (buff[0] == 'P') { /* position */
                                    if (val != 0.0 && fabs(val - 999999.999999) >= 1e-6)
                                        {
                                            peph.pos[sat-1][j] = val*(j < 3 ? 1000.0 : 1e-6);
                                            v = 1; /* valid epoch */
                                        }
                                    if ((base = bfact[j < 3 ? 0 : 1]) > 0.0 && std > 0.0)
                                        {
                                            peph.std[sat-1][j] = (float)(std::pow(base, std)*(j < 3 ? 1e-3 : 1e-12));
                                        }
                            }
                            else if (v) { /* velocity */
                                    if (val != 0.0 && fabs(val - 999999.999999) >= 1e-6)
                                        {
                                            peph.vel[sat-1][j] = val*(j < 3 ? 0.1 : 1e-10);
                                        }
                                    if ((base = bfact[j < 3 ? 0 : 1]) > 0.0 && std > 0.0)
                                        {
                                            peph.vst[sat-1][j] = (float)(std::pow(base, std)*(j < 3 ? 1e-7 : 1e-16));
                                        }
                            }
                        }
                }
            if (v)
                {
                    if (!addpeph(nav, &peph)) return;
                }
        }
}


/* compare precise ephemeris -------------------------------------------------*/
int cmppeph(const void *p1,  const void *p2)
{
    peph_t *q1 = (peph_t *)p1, *q2 = (peph_t *)p2;
    double tt = timediff(q1->time, q2->time);
    return tt < -1e-9 ? -1 : (tt > 1e-9 ? 1 : q1->index - q2->index);
}


/* combine precise ephemeris -------------------------------------------------*/
void combpeph(nav_t *nav,  int opt)
{
    int i, j, k, m;

    trace(3, "combpeph: ne=%d\n", nav->ne);

    qsort(nav->peph, nav->ne, sizeof(peph_t), cmppeph);

    if (opt&4) return;

    for (i = 0, j = 1; j < nav->ne; j++)
        {
            if (fabs(timediff(nav->peph[i].time, nav->peph[j].time)) < 1e-9)
                {
                    for (k = 0; k < MAXSAT; k++)
                        {
                            if (norm_rtk(nav->peph[j].pos[k], 4) <= 0.0) continue;
                            for (m = 0;m < 4; m++) nav->peph[i].pos[k][m] = nav->peph[j].pos[k][m];
                            for (m = 0;m < 4; m++) nav->peph[i].std[k][m] = nav->peph[j].std[k][m];
                            for (m = 0;m < 4; m++) nav->peph[i].vel[k][m] = nav->peph[j].vel[k][m];
                            for (m = 0;m < 4; m++) nav->peph[i].vst[k][m] = nav->peph[j].vst[k][m];
                        }
                }
            else if (++i < j) nav->peph[i] = nav->peph[j];
        }
    nav->ne = i+1;

    trace(4, "combpeph: ne=%d\n", nav->ne);
}


/* read sp3 precise ephemeris file ---------------------------------------------
 * read sp3 precise ephemeris/clock files and set them to navigation data
 * args   : char   *file       I   sp3-c precise ephemeris file
 *                                 (wind-card * is expanded)
 *          nav_t  *nav        IO  navigation data
 *          int    opt         I   options (1: only observed + 2: only predicted +
 *                                 4: not combined)
 * return : none
 * notes  : see ref [1]
 *          precise ephemeris is appended and combined
 *          nav->peph and nav->ne must by properly initialized before calling the
 *          function
 *          only files with extensions of .sp3, .SP3, .eph* and .EPH* are read
 *-----------------------------------------------------------------------------*/
void readsp3(const char *file, nav_t *nav, int opt)
{
    FILE *fp;
    gtime_t time = {0, 0};
    double bfact[2] = {};
    int i, j, n, ns, sats[MAXSAT] = {};
    char *efiles[MAXEXFILE], *ext, type = ' ', tsys[4] = "";

    trace(3, "readpephs: file=%s\n", file);

    for (i = 0; i < MAXEXFILE; i++)
        {
            if (!(efiles[i] = (char *)malloc(1024)))
                {
                    for (i--; i >= 0; i--) free(efiles[i]);
                    return;
                }
        }
    /* expand wild card in file path */
    n = expath(file, efiles, MAXEXFILE);

    for (i = j = 0; i < n; i++)
        {
            if (!(ext = strrchr(efiles[i], '.'))) continue;

            if (!strstr(ext + 1, "sp3") && !strstr(ext + 1, ".SP3") &&
                    !strstr(ext + 1, "eph") && !strstr(ext + 1, ".EPH")) continue;

            if (!(fp = fopen(efiles[i], "r")))
                {
                    trace(2, "sp3 file open error %s\n", efiles[i]);
                    continue;
                }
            /* read sp3 header */
            ns = readsp3h(fp, &time, &type, sats, bfact, tsys);

            /* read sp3 body */
            readsp3b(fp, type, sats, ns, bfact, tsys, j++, opt, nav);

            fclose(fp);
        }
    for (i = 0; i < MAXEXFILE; i++) free(efiles[i]);

    /* combine precise ephemeris */
    if (nav->ne > 0) combpeph(nav, opt);
}


/* read satellite antenna parameters -------------------------------------------
 * read satellite antenna parameters
 * args   : char   *file       I   antenna parameter file
 *          gtime_t time       I   time
 *          nav_t  *nav        IO  navigation data
 * return : status (1:ok,0:error)
 * notes  : only support antex format for the antenna parameter file
 *-----------------------------------------------------------------------------*/
int readsap(const char *file, gtime_t time, nav_t *nav)
{
    pcvs_t pcvs = {0, 0, (pcv_t*){ 0 } };
    pcv_t pcv0 = { 0, {}, {}, {0,0}, {0,0}, {{},{}}, {{},{}} }, *pcv;
    int i;

    trace(3, "readsap : file=%s time=%s\n", file, time_str(time, 0));

    if (!readpcv(file, &pcvs)) return 0;

    for (i = 0; i < MAXSAT; i++)
        {
            pcv = searchpcv(i + 1, "", time, &pcvs);
            nav->pcvs[i] = pcv ? *pcv : pcv0;
        }
    free(pcv);
    return 1;
}


/* read dcb parameters file --------------------------------------------------*/
int readdcbf(const char *file, nav_t *nav, const sta_t *sta)
{
    FILE *fp;
    double cbias;
    char buff[256], str1[32], str2[32] = "";
    int i, j, sat, type = 0;

    trace(3, "readdcbf: file=%s\n", file);

    if (!(fp = fopen(file, "r")))
        {
            trace(2, "dcb parameters file open error: %s\n", file);
            return 0;
        }
    while (fgets(buff, sizeof(buff), fp))
        {
            if      (strstr(buff, "DIFFERENTIAL (P1-P2) CODE BIASES")) type = 1;
            else if (strstr(buff, "DIFFERENTIAL (P1-C1) CODE BIASES")) type = 2;
            else if (strstr(buff, "DIFFERENTIAL (P2-C2) CODE BIASES")) type = 3;

            if (!type || sscanf(buff, "%s %s", str1, str2) < 1) continue;

            if ((cbias = str2num(buff, 26, 9)) == 0.0) continue;

            if (sta && (!strcmp(str1, "G") || !strcmp(str1, "R")))
                { /* receiver dcb */
                    for (i = 0; i < MAXRCV; i++)
                        {
                            if (!strcmp(sta[i].name, str2)) break;
                        }
                    if (i < MAXRCV)
                        {
                            j = !strcmp(str1, "G") ? 0 : 1;
                            nav->rbias[i][j][type-1] = cbias * 1e-9 * SPEED_OF_LIGHT; /* ns -> m */
                        }
                }
            else if ((sat = satid2no(str1)))
                { /* satellite dcb */
                    nav->cbias[sat-1][type-1] = cbias * 1e-9 * SPEED_OF_LIGHT; /* ns -> m */
                }
        }
    fclose(fp);

    return 1;
}


/* read dcb parameters ---------------------------------------------------------
 * read differential code bias (dcb) parameters
 * args   : char   *file       I   dcb parameters file (wild-card * expanded)
 *          nav_t  *nav        IO  navigation data
 *          sta_t  *sta        I   station info data to inport receiver dcb
 *                                 (NULL: no use)
 * return : status (1:ok,0:error)
 * notes  : currently only p1-c1 bias of code *.dcb file
 *-----------------------------------------------------------------------------*/
int readdcb(const char *file, nav_t *nav, const sta_t *sta)
{
    int i, j, n;
    char *efiles[MAXEXFILE] = {};

    trace(3, "readdcb : file=%s\n", file);

    for (i = 0;i < MAXSAT; i++) for (j = 0; j < 3; j++)
        {
            nav->cbias[i][j] = 0.0;
        }
    for (i = 0; i < MAXEXFILE; i++)
        {
            if (!(efiles[i] = (char *)malloc(1024)))
                {
                    for (i--; i >= 0; i--) free(efiles[i]);
                    return 0;
                }
        }
    n = expath(file, efiles, MAXEXFILE);

    for (i = 0; i < n; i++)
        {
            readdcbf(efiles[i], nav, sta);
        }
    for (i = 0; i < MAXEXFILE;i ++) free(efiles[i]);

    return 1;
}


/* add satellite fcb ---------------------------------------------------------*/
int addfcb(nav_t *nav, gtime_t ts, gtime_t te, int sat,
        const double *bias, const double *std)
{
    fcbd_t *nav_fcb;
    int i, j;

    if (nav->nf > 0 && fabs(timediff(ts, nav->fcb[nav->nf-1].ts)) <= 1e-3)
        {
            for (i = 0; i < 3; i++)
                {
                    nav->fcb[nav->nf-1].bias[sat-1][i] = bias[i];
                    nav->fcb[nav->nf-1].std [sat-1][i] = std [i];
                }
            return 1;
        }
    if (nav->nf >= nav->nfmax)
        {
            nav->nfmax = nav->nfmax <= 0 ? 2048 : nav->nfmax * 2;
            if (!(nav_fcb = (fcbd_t *)realloc(nav->fcb, sizeof(fcbd_t)*nav->nfmax)))
                {
                    free(nav->fcb);
                    nav->nf = nav->nfmax = 0;
                    return 0;
                }
            nav->fcb = nav_fcb;
        }
    for (i = 0; i < MAXSAT; i++) for (j = 0; j < 3; j++)
        {
            nav->fcb[nav->nf].bias[i][j] = nav->fcb[nav->nf].std[i][j] = 0.0;
        }
    for (i = 0; i < 3; i++)
        {
            nav->fcb[nav->nf].bias[sat-1][i] = bias[i];
            nav->fcb[nav->nf].std [sat-1][i] = std [i];
        }
    nav->fcb[nav->nf  ].ts = ts;
    nav->fcb[nav->nf++].te = te;
    return 1;
}


/* read satellite fcb file ---------------------------------------------------*/
int readfcbf(const char *file, nav_t *nav)
{
    FILE *fp;
    gtime_t ts, te;
    double ep1[6], ep2[6], bias[3] = {}, std[3] = {};
    char buff[1024], str[32], *p;
    int sat;

    trace(3, "readfcbf: file=%s\n", file);

    if (!(fp = fopen(file, "r")))
        {
            trace(2, "fcb parameters file open error: %s\n", file);
            return 0;
        }
    while (fgets(buff, sizeof(buff), fp))
        {
            if ((p = strchr(buff, '#'))) *p = '\0';
            if (sscanf(buff, "%lf/%lf/%lf %lf:%lf:%lf %lf/%lf/%lf %lf:%lf:%lf %s"
                    "%lf %lf %lf %lf %lf %lf", ep1, ep1+1, ep1+2, ep1+3, ep1+4, ep1+5,
                    ep2, ep2+1, ep2+2, ep2+3, ep2+4, ep2+5, str, bias, std, bias+1, std+1,
                    bias+2, std+2) < 17) continue;
            if (!(sat = satid2no(str))) continue;
            ts = epoch2time(ep1);
            te = epoch2time(ep2);
            if (!addfcb(nav, ts, te, sat, bias, std))
                {
                    fclose(fp);
                    return 0;
                }
        }
    fclose(fp);
    return 1;
}


/* compare satellite fcb -----------------------------------------------------*/
int cmpfcb(const void *p1, const void *p2)
{
    fcbd_t *q1 = (fcbd_t *)p1, *q2 = (fcbd_t *)p2;
    double tt = timediff(q1->ts, q2->ts);
    return tt < -1e-3 ? -1 : (tt > 1e-3 ? 1 : 0);
}


/* read satellite fcb data -----------------------------------------------------
 * read satellite fractional cycle bias (dcb) parameters
 * args   : char   *file       I   fcb parameters file (wild-card * expanded)
 *          nav_t  *nav        IO  navigation data
 * return : status (1:ok,0:error)
 * notes  : fcb data appended to navigation data
 *-----------------------------------------------------------------------------*/
int readfcb(const char *file, nav_t *nav)
{
    char *efiles[MAXEXFILE] = {};
    int i, n;

    trace(3, "readfcb : file=%s\n", file);

    for (i = 0; i < MAXEXFILE; i++)
        {
            if (!(efiles[i] = (char *)malloc(1024)))
                {
                    for (i--; i >= 0; i--) free(efiles[i]);
                    return 0;
                }
        }
    n = expath(file, efiles, MAXEXFILE);

    for (i = 0; i < n; i++)
        {
            readfcbf(efiles[i], nav);
        }
    for (i = 0; i < MAXEXFILE; i++) free(efiles[i]);

    if (nav->nf > 1)
        {
            qsort(nav->fcb, nav->nf, sizeof(fcbd_t), cmpfcb);
        }
    return 1;
}


/* polynomial interpolation by Neville's algorithm ---------------------------*/
double interppol(const double *x, double *y, int n)
{
    int i, j;

    for (j = 1; j < n; j++)
        {
            for (i = 0; i < n - j; i++)
                {
                    y[i] = (x[i+j] * y[i] - x[i] * y[i+1]) / (x[i+j] - x[i]);
                }
        }
    return y[0];
}


/* satellite position by precise ephemeris -----------------------------------*/
int pephpos(gtime_t time, int sat, const nav_t *nav, double *rs,
        double *dts, double *vare, double *varc)
{
    double t[NMAX+1], p[3][NMAX+1], c[2], *pos, std = 0.0, s[3], sinl, cosl;
    int i, j, k, index;

    trace(4, "pephpos : time=%s sat=%2d\n", time_str(time, 3), sat);

    rs[0] = rs[1] = rs[2] = dts[0] = 0.0;

    if (nav->ne < NMAX + 1 ||
            timediff(time, nav->peph[0].time) < -MAXDTE ||
            timediff(time, nav->peph[nav->ne-1].time) > MAXDTE)
        {
            trace(3, "no prec ephem %s sat=%2d\n", time_str(time, 0), sat);
            return 0;
        }
    /* binary search */
    for (i = 0, j = nav->ne - 1; i < j;)
        {
            k = (i + j) / 2;
            if (timediff(nav->peph[k].time, time) < 0.0) i = k + 1; else j = k;
        }
    index = i <= 0 ? 0 : i-1;

    /* polynomial interpolation for orbit */
    i = index - (NMAX + 1) / 2;
    if (i < 0) i = 0; else if (i + NMAX >= nav->ne) i = nav->ne - NMAX - 1;

    for (j = 0; j <= NMAX; j++)
        {
            t[j] = timediff(nav->peph[i+j].time, time);
            if (norm_rtk(nav->peph[i+j].pos[sat-1], 3) <= 0.0)
                {
                    trace(3, "prec ephem outage %s sat=%2d\n", time_str(time, 0), sat);
                    return 0;
                }
        }
    for (j = 0;j<=NMAX;j++)
        {
            pos = nav->peph[i+j].pos[sat-1];
#if 0
            p[0][j] = pos[0];
            p[1][j] = pos[1];
#else
            /* correciton for earh rotation ver.2.4.0 */
            sinl = sin(DEFAULT_OMEGA_EARTH_DOT * t[j]);
            cosl = cos(DEFAULT_OMEGA_EARTH_DOT * t[j]);
            p[0][j] = cosl * pos[0] - sinl * pos[1];
            p[1][j] = sinl * pos[0] + cosl * pos[1];
#endif
            p[2][j] = pos[2];
        }
    for (i = 0; i < 3; i++)
        {
            rs[i] = interppol(t, p[i], NMAX + 1);
        }
    if (vare)
        {
            for (i = 0; i < 3; i++) s[i] = nav->peph[index].std[sat-1][i];
            std = norm_rtk(s, 3);

            /* extrapolation error for orbit */
            if      (t[0   ] > 0.0) std += EXTERR_EPH * std::pow(t[0   ], 2.0) / 2.0;
            else if (t[NMAX] < 0.0) std += EXTERR_EPH * std::pow(t[NMAX], 2.0) / 2.0;
            *vare = std::pow(std, 2.0);
        }
    /* linear interpolation for clock */
    t[0] = timediff(time, nav->peph[index  ].time);
    t[1] = timediff(time, nav->peph[index+1].time);
    c[0] = nav->peph[index  ].pos[sat-1][3];
    c[1] = nav->peph[index+1].pos[sat-1][3];

    if (t[0] <= 0.0)
        {
            if ((dts[0] = c[0]) != 0.0)
                {
                    std = nav->peph[index].std[sat-1][3] * SPEED_OF_LIGHT - EXTERR_CLK * t[0];
                }
        }
    else if (t[1] >= 0.0)
        {
            if ((dts[0] = c[1]) != 0.0)
                {
                    std = nav->peph[index+1].std[sat-1][3] * SPEED_OF_LIGHT + EXTERR_CLK * t[1];
                }
        }
    else if (c[0] != 0.0 && c[1] != 0.0)
        {
            dts[0] = (c[1] * t[0] - c[0] * t[1]) / (t[0] - t[1]);
            i = t[0] < -t[1] ? 0: 1;
            std = nav->peph[index+i].std[sat-1][3] + EXTERR_CLK * fabs(t[i]);
        }
    else
        {
            dts[0] = 0.0;
        }
    if (varc) *varc = std::pow(std, 2.0);
    return 1;
}


/* satellite clock by precise clock ------------------------------------------*/
int pephclk(gtime_t time, int sat, const nav_t *nav, double *dts,
        double *varc)
{
    double t[2], c[2], std;
    int i, j, k, index;

    trace(4, "pephclk : time=%s sat=%2d\n", time_str(time, 3), sat);

    if (nav->nc < 2 ||
            timediff(time, nav->pclk[0].time) < -MAXDTE ||
            timediff(time, nav->pclk[nav->nc-1].time) > MAXDTE)
        {
            trace(3, "no prec clock %s sat=%2d\n", time_str(time, 0), sat);
            return 1;
        }
    /* binary search */
    for (i = 0, j = nav->nc - 1; i < j;)
        {
            k = (i + j) / 2;
            if (timediff(nav->pclk[k].time, time) < 0.0) i = k + 1; else j = k;
        }
    index = i<=0?0:i-1;

    /* linear interpolation for clock */
    t[0] = timediff(time, nav->pclk[index  ].time);
    t[1] = timediff(time, nav->pclk[index+1].time);
    c[0] = nav->pclk[index  ].clk[sat-1][0];
    c[1] = nav->pclk[index+1].clk[sat-1][0];

    if (t[0] <= 0.0)
        {
            if ((dts[0] = c[0]) == 0.0) return 0;
            std = nav->pclk[index].std[sat-1][0] * SPEED_OF_LIGHT - EXTERR_CLK * t[0];
        }
    else if (t[1] >= 0.0)
        {
            if ((dts[0] = c[1]) == 0.0) return 0;
            std = nav->pclk[index+1].std[sat-1][0] * SPEED_OF_LIGHT + EXTERR_CLK * t[1];
        }
    else if (c[0] != 0.0 && c[1] != 0.0)
        {
            dts[0] = (c[1]*t[0]-c[0]*t[1]) / (t[0] - t[1]);
            i = t[0] < -t[1] ? 0 : 1;
            std = nav->pclk[index+i].std[sat-1][0] * SPEED_OF_LIGHT + EXTERR_CLK * fabs(t[i]);
        }
    else
        {
            trace(3, "prec clock outage %s sat=%2d\n", time_str(time, 0), sat);
            return 0;
        }
    if (varc) *varc = std::pow(std, 2.0);
    return 1;
}


/* satellite antenna phase center offset ---------------------------------------
 * compute satellite antenna phase center offset in ecef
 * args   : gtime_t time       I   time (gpst)
 *          double *rs         I   satellite position and velocity (ecef)
 *                                 {x,y,z,vx,vy,vz} (m|m/s)
 *          int    sat         I   satellite number
 *          nav_t  *nav        I   navigation data
 *          double *dant       I   satellite antenna phase center offset (ecef)
 *                                 {dx,dy,dz} (m) (iono-free LC value)
 * return : none
 *-----------------------------------------------------------------------------*/
void satantoff(gtime_t time, const double *rs, int sat, const nav_t *nav,
        double *dant)
{
    const double *lam = nav->lam[sat-1];
    const pcv_t *pcv = nav->pcvs+sat-1;
    double ex[3], ey[3], ez[3], es[3], r[3], rsun[3], gmst, erpv[5] = {};
    double gamma, C1, C2, dant1, dant2;
    int i, j = 0, k = 1;

    trace(4, "satantoff: time=%s sat=%2d\n", time_str(time, 3), sat);

    /* sun position in ecef */
    sunmoonpos(gpst2utc(time), erpv, rsun, NULL, &gmst);

    /* unit vectors of satellite fixed coordinates */
    for (i = 0; i < 3; i++) r[i] = -rs[i];
    if (!normv3(r, ez)) return;
    for (i = 0; i < 3; i++) r[i] = rsun[i]-rs[i];
    if (!normv3(r, es)) return;
    cross3(ez, es, r);
    if (!normv3(r, ey)) return;
    cross3(ey, ez, ex);

    if (NFREQ >= 3 && (satsys(sat, NULL) & (SYS_GAL | SYS_SBS))) k = 2;

    if (NFREQ < 2 || lam[j] == 0.0 || lam[k] == 0.0) return;

    gamma = std::pow(lam[k], 2.0) / std::pow(lam[j], 2.0);
    C1 = gamma / (gamma - 1.0);
    C2 = -1.0 / (gamma - 1.0);

    /* iono-free LC */
    for (i = 0; i < 3; i++)
        {
            dant1 = pcv->off[j][0] * ex[i] + pcv->off[j][1] * ey[i] + pcv->off[j][2] * ez[i];
            dant2 = pcv->off[k][0] * ex[i] + pcv->off[k][1] * ey[i] + pcv->off[k][2] * ez[i];
            dant[i] = C1 * dant1 + C2 * dant2;
        }
}


/* satellite position/clock by precise ephemeris/clock -------------------------
 * compute satellite position/clock with precise ephemeris/clock
 * args   : gtime_t time       I   time (gpst)
 *          int    sat         I   satellite number
 *          nav_t  *nav        I   navigation data
 *          int    opt         I   sat postion option
 *                                 (0: center of mass, 1: antenna phase center)
 *          double *rs         O   sat position and velocity (ecef)
 *                                 {x,y,z,vx,vy,vz} (m|m/s)
 *          double *dts        O   sat clock {bias,drift} (s|s/s)
 *          double *var        IO  sat position and clock error variance (m)
 *                                 (NULL: no output)
 * return : status (1:ok,0:error or data outage)
 * notes  : clock includes relativistic correction but does not contain code bias
 *          before calling the function, nav->peph, nav->ne, nav->pclk and
 *          nav->nc must be set by calling readsp3(), readrnx() or readrnxt()
 *          if precise clocks are not set, clocks in sp3 are used instead
 *-----------------------------------------------------------------------------*/
int peph2pos(gtime_t time, int sat, const nav_t *nav, int opt,
        double *rs, double *dts, double *var)
{
    double rss[3], rst[3], dtss[1], dtst[1], dant[3] = {}, vare = 0.0, varc = 0.0, tt = 1e-3;
    int i;

    trace(4, "peph2pos: time=%s sat=%2d opt=%d\n", time_str(time, 3), sat, opt);

    if (sat <= 0 || MAXSAT < sat) return 0;

    /* satellite position and clock bias */
    if (!pephpos(time, sat, nav, rss, dtss, &vare, &varc) ||
            !pephclk(time, sat, nav, dtss, &varc)) return 0;

    time = timeadd(time, tt);
    if (!pephpos(time, sat, nav, rst, dtst, NULL, NULL) ||
            !pephclk(time, sat, nav, dtst, NULL)) return 0;

    /* satellite antenna offset correction */
    if (opt)
        {
            satantoff(time, rss, sat, nav, dant);
        }
    for (i = 0;i<3;i++)
        {
            rs[i  ] = rss[i] + dant[i];
            rs[i+3] = (rst[i] - rss[i]) / tt;
        }
    /* relativistic effect correction */
    if (dtss[0] != 0.0)
        {
            dts[0] = dtss[0] - 2.0 * dot(rs, rs+3, 3) / SPEED_OF_LIGHT / SPEED_OF_LIGHT;
            dts[1] = (dtst[0] - dtss[0]) / tt;
        }
    else
        { /* no precise clock */
            dts[0] = dts[1] = 0.0;
        }
    if (var) *var = vare + varc;

    return 1;
}
